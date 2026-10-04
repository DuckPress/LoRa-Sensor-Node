/*
 * Lilygo T3-S3 v1.2 — Sensor Node Firmware  (version: FIRMWARE_VERSION, config.h)
 *
 * Sensors  : HLK-LD2413 24 GHz radar (UART: ESP RX=GPIO44, TX=GPIO43)  [primary]
 *            RCWL-1670 ultrasonic    (TRIG=GPIO41, ECHO=GPIO42)        [cross-check]
 *            SHT3x temp/humidity (I2C Wire1, SDA=GPIO16, SCL=GPIO15)
 *            DS3231 RTC          (I2C Wire1, shared bus)
 *            optional LIS2DW12 tilt + BMP/BME280 pressure (Wire1, auto-detected)
 *
 * Storage  : Onboard MicroSD (SPI FSPI, default bus)
 * Display  : Onboard SSD1306 128×64 OLED (I2C Wire, SDA=18, SCL=17)
 *
 * Comms    : (1) LoRa SX1262 → XIAO ESP32-C3 + Wio-SX1262 gateway  [PRIMARY]
 *            (2) WiFi → Google Apps Script HTTPS                      [SECONDARY]
 *            (3) OTA via GitHub Releases                              [MAINTENANCE]
 *
 * Power    : Deep sleep between readings — SLEEP_DURATION_US cadence
 *            (SURVEY_SLEEP_US in survey mode). WiFi only every
 *            WIFI_UPLOAD_EVERY_N wakes, backing off to WIFI_BACKOFF_EVERY_N
 *            while no network answers; LoRa retries back off while the
 *            gateway is silent. LiPo < BAT_CUTOFF_V skips radio TX entirely.
 *
 * Distance : every sensor present is read each wake: burst → trimmed mean
 *            (the ultrasonic also speed-of-sound corrected) → that sensor's
 *            own 1-D Kalman filter (Q scaled by the real sleep). The radar is
 *            the primary; the ultrasonic stands in when a radar burst is empty.
 *
 * PARTITION SCHEME (required for OTA):
 *   Tools → Partition Scheme → Minimal SPIFFS (1.9MB APP/190KB SPIFFS)
 */

#include "config.h"
#include "wifi_manager.h"
#include "sensors.h"
#include "sd_logger.h"
#include "secret_store.h"   // WiFi/GAS/LoRa-token creds — read from SD, not the binary
#include "display_mgr.h"
#include "uploader.h"
#include "gas_upload.h"   // gasFetch() — remote node config (survey mode)
#include "lora_comms.h"
#include "ota_updater.h"
#include "pending.h"
#include "crashlog.h"      // stage breadcrumbs + core-dump summary capture (H2)
#include <esp_sleep.h>

#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <esp_system.h>   // esp_reset_reason()
#include <Preferences.h>  // NVS boot-loop counter (survives brownout/panic)
#include <sys/time.h>     // gettimeofday() — measures an early (motion) wake

// ================================================================
//  RTC RAM — persists across deep sleep
// ================================================================
RTC_DATA_ATTR static uint32_t s_wakeCount      = 0;

// Epoch fallback: reconstruct ISO-8601 timestamps when the DS3231 is
// unavailable by advancing the last known good epoch by elapsed sleep time.
RTC_DATA_ATTR static uint32_t s_lastKnownEpoch = 0;
RTC_DATA_ATTR static uint32_t s_elapsedEstSecs = 0;   // seconds since last good RTC read
RTC_DATA_ATTR static uint32_t s_lastSleepSecs  = 30;  // actual duration of previous wake

// Adaptive sleep: previous wake's RAW distance — drives the wake-rate decision
// so the node reacts to real surface activity, not the filter's own settling.
RTC_DATA_ATTR static float    s_prevDistanceCm = -1.0f;

// Last WiFi RSSI — included in LoRa payload and SD log.
RTC_DATA_ATTR static int32_t  s_lastWifiRssi   = 0;

// Survey mode (bathymetric-reference operation): fixed wake cadence + long
// radar burst. Toggled remotely from the Config sheet (fetched on WiFi wakes);
// persists across deep sleep here. Reset by a reboot — wake #1 is always a
// WiFi wake, so the flag is re-fetched within the first wake after any reset.
RTC_DATA_ATTR static bool     s_surveyMode     = false;

// True while an SD backlog is draining. It NO LONGER gates LoRa TX (the node
// keeps transmitting so it recovers to the gateway the instant the link comes
// back). It only orders the WiFi-upload path: while a backlog exists, a
// LoRa-unconfirmed current reading is appended to the queue rather than
// direct-uploaded, so it can't jump ahead of the older queued rows on the WiFi
// path. Cloud consumers sort by timestamp, so out-of-order arrival is harmless.
RTC_DATA_ATTR static bool     s_backlogPending = false;

// Keep the requested cadence measured from the beginning of each wake.
static uint32_t s_wakeStartedMs = 0;

// Consecutive wakes the node's LoRa TX went unacknowledged — a persistent count
// means the gateway is unreachable (down, or an RF/antenna fault on its side).
// Surfaced on serial/OLED for on-site diagnosis; the cloud sees the same
// outage as a stalled gateway heartbeat (Sheets health-watch). RTC RAM so it
// spans deep-sleep wakes; a reset (which zeroes it) is itself a fresh start.
RTC_DATA_ATTR static uint32_t s_consecutiveNoAck = 0;

// Consecutive WiFi wakes whose connect failed. Past WIFI_FAIL_BACKOFF_AFTER the
// node only tries WiFi every WIFI_BACKOFF_EVERY_N wakes (see config.h); one
// success, or any reset, restores the normal interval.
RTC_DATA_ATTR static uint8_t  s_wifiFailStreak = 0;

// Wake count of the last Config-requested OTA check (0 = none since reset), so
// a standing ota_update_requested=1 costs one GitHub check per
// OTA_REQUEST_MIN_WAKES instead of one on every WiFi wake.
RTC_DATA_ATTR static uint32_t s_lastOtaReqWake = 0;

// Set once this node has applied a remote tilt-baseline reset; the next nodecfg
// poll then carries &trd=1 and the cloud clears the one-shot only on that
// confirmation (a lost response can't swallow the reset).
RTC_DATA_ATTR static bool     s_tiltResetAck   = false;

// Motion-wake rate limit: after a motion (EXT0) wake the accelerometer interrupt
// stays disarmed for MOTION_WAKE_HOLDOFF_WAKES wakes.
RTC_DATA_ATTR static uint8_t  s_motionHoldoff  = 0;

// System time (µs, gettimeofday) right before the last deep sleep. The RTC timer
// keeps system time running through deep sleep, so after an EARLY (motion) wake
// the real sleep length is now − this, not the programmed duration.
RTC_DATA_ATTR static int64_t  s_sleepEnterUs   = 0;

// Which sensor produced s_prevDistanceCm (the adaptive-sleep delta only compares
// a sensor with itself).
RTC_DATA_ATTR static uint8_t  s_prevSensor     = 0;

// Set from the NVS boot-loop counter at the top of setup(): true means the node
// has rebooted without completing a wake too many times in a row and should run
// this wake in SAFE MODE (no radio, SD-log only, long recovery sleep).
static bool s_safeMode = false;

// Remote recovery knobs fetched from the Config sheet (?action=nodecfg) on WiFi
// wakes. pause_flush halts backlog draining; flush_cap (0 = auto/voltage-scaled)
// hard-limits entries per wake. Let an operator throttle a struggling field
// node from the sheet without a reflash or a site visit. RTC RAM: they must
// still hold on the non-WiFi wakes, which is where the LoRa backlog flush runs.
RTC_DATA_ATTR static bool     s_pauseFlush = false;
RTC_DATA_ATTR static uint32_t s_flushCap   = 0;

// Current system time in µs (see s_sleepEnterUs).
static int64_t nowUs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (int64_t)tv.tv_sec * 1000000LL + (int64_t)tv.tv_usec;
}

// OTA downloads ~1.3 MB over WiFi — only with a healthy resting cell (or none:
// an implausible reading is USB-only operation).
static bool otaBatteryOk(float v) {
  return v <= BAT_PLAUSIBLE_MIN_V || v > BAT_PLAUSIBLE_MAX_V || v >= OTA_MIN_BAT_V;
}

// ================================================================
//  Graceful shutdown helpers — called before every deep sleep
// ================================================================
static void shutdownPeripherals() {
  displayOff();       // OLED panel off (~7–8 mA → µA) — MUST precede Wire.end()
  loraShutdown();     // SX1262 → sleep (~0.5 µA vs ~1.5 mA standby)
  sensorsShutdown();  // end Wire1 (sensor I2C bus, GPIO15/16)
  Wire.end();         // end Wire  (OLED I2C bus, GPIO17/18)
}

// ================================================================
//  Reset-reason → string (for boot diagnostics)
// ================================================================
static const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

// ================================================================
//  Boot-loop safe mode — NVS-backed detection that survives a reset
//
//  RTC RAM (s_wakeCount) is wiped by a brownout/panic, so it can't tell "this
//  is the 6th crash in a row" from "first boot". NVS/flash survives, so the
//  crash count lives there. checkBootLoop() runs near the top of setup() and
//  only counts REAL reboots (a deep-sleep wake is normal, not a crash);
//  markBootConfirmed() runs once a wake reaches the end, clearing the count.
//  A run of unconfirmed reboots therefore means the node keeps dying mid-wake
//  → enter safe mode for one cycle (see config.h SAFE_MODE_*).
// ================================================================
static void checkBootLoop(esp_reset_reason_t reason) {
  if (reason == ESP_RST_DEEPSLEEP) return;   // normal wake, not a reboot

  Preferences prefs;
  prefs.begin("boot", false);
  uint32_t n = prefs.getUInt("unconf", 0) + 1;
  prefs.putUInt("unconf", n);
  prefs.end();

  if (n >= SAFE_MODE_BOOT_THRESHOLD) {
    s_safeMode = true;
    Serial.printf("[SafeMode] %lu unconfirmed reboots in a row — SAFE MODE: "
                  "skipping all radio work, SD-logging only, %llus recovery "
                  "sleep so the cell recovers.\n",
                  (unsigned long)n, SAFE_MODE_SLEEP_US / 1000000ULL);
  } else if (n > 1) {
    Serial.printf("[SafeMode] Unconfirmed reboot #%lu (safe mode at %lu)\n",
                  (unsigned long)n, (unsigned long)SAFE_MODE_BOOT_THRESHOLD);
  }
}

// Clear the NVS unconfirmed-reboot counter — this wake ran to completion.
// Called right before every deep sleep (normal AND safe-mode), so a safe-mode
// recovery cycle also counts as "confirmed": the node retries the normal path
// after one clean recovery sleep instead of staying latched in safe mode.
static void markBootConfirmed() {
  Preferences prefs;
  prefs.begin("boot", false);
  if (prefs.getUInt("unconf", 0) != 0) prefs.putUInt("unconf", 0);
  prefs.end();
}

// ================================================================
//  Deep sleep
// ================================================================
static void enterDeepSleep(uint64_t sleepUs) {
  markBootConfirmed();   // reached a clean shutdown — this wake is confirmed
  s_lastSleepSecs = (uint32_t)(sleepUs / 1000000ULL);
  // Count this wake's AWAKE time toward the epoch estimate too — sleep alone
  // under-counts by the 5–20 s each wake spends awake, which accumulates into
  // hours of timestamp drift whenever the DS3231 is unavailable for long.
  s_elapsedEstSecs += (millis() + 500) / 1000;
  Serial.printf("[Sleep] Wake #%lu done — sleeping %lu s\n",
                (unsigned long)s_wakeCount,
                (unsigned long)s_lastSleepSecs);
  Serial.flush();
  crashStageSet(STAGE_SLEEP);
  // Arm the accelerometer's wake-up interrupt BEFORE the I2C bus is released
  // (no-op when no LIS2DW12 / PIN_TILT_INT is fitted) — unless a recent motion
  // wake put it on hold-off, so a vibrating mount can't keep re-waking the
  // node. The timer wake below stays armed either way; whichever fires first
  // wakes the node.
  if (s_motionHoldoff > 0) s_motionHoldoff--;
  else                     tiltArmMotionWake();
  shutdownPeripherals();
  s_sleepEnterUs = nowUs();
  esp_deep_sleep(sleepUs);
}

// The configured interval includes sensor and radio work.  If a wake overruns
// the interval, retain a short sleep instead of entering a reset loop.
static uint64_t cadenceSleepUs(uint64_t cadenceUs) {
  const uint64_t awakeUs = (uint64_t)(millis() - s_wakeStartedMs) * 1000ULL;
  constexpr uint64_t MIN_SLEEP_US = 1000000ULL;
  return (awakeUs + MIN_SLEEP_US >= cadenceUs) ? MIN_SLEEP_US
                                                 : cadenceUs - awakeUs;
}

// ================================================================
//  Debug / upload mode
//
//  Deep sleep powers down the T3-S3's native USB CDC, so the serial port
//  only exists for the few seconds the node is awake each wake. Holding
//  PIN_DEBUG_BTN LOW at boot short-circuits straight to an idle loop here —
//  before the watchdog is armed and before anything touches SPI/I2C/WiFi —
//  so USB stays enumerated indefinitely and there's no race to catch the
//  port before the node sleeps again. Returns once the button is released
//  or after DEBUG_MODE_TIMEOUT_MS, at which point setup() continues as a
//  normal wake (so the node doesn't idle forever on a forgotten button).
// ================================================================
static void checkDebugButton() {
  pinMode(PIN_DEBUG_BTN, INPUT_PULLUP);
  delay(20);                                  // let the pull-up settle
  if (digitalRead(PIN_DEBUG_BTN) != LOW) return;   // not held — normal boot

  Serial.println(F("\n=== DEBUG MODE — button held at boot ==="));
  Serial.println(F("[Debug] Sensor/LoRa/SD/WiFi pipeline skipped."));
  Serial.println(F("[Debug] USB stays up — safe to upload new firmware now."));
  Serial.printf("[Debug] Release the button, or wait, to resume normal "
                "operation (auto-resumes after %lu s either way).\n",
                (unsigned long)(DEBUG_MODE_TIMEOUT_MS / 1000));

  if (DISPLAY_ENABLED) {
    displayInit();
    displaySplash("DEBUG MODE", "Ready to flash");
  }

  uint32_t debugStart = millis();
  while (digitalRead(PIN_DEBUG_BTN) == LOW &&
         millis() - debugStart < DEBUG_MODE_TIMEOUT_MS) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(F("\n[Debug] Resuming normal operation"));
  displayOff();
}

// ================================================================
//  setup() — all work happens here; loop() is never reached
// ================================================================
void setup() {
  s_wakeStartedMs = millis();
  s_wakeCount++;

  // How long did we actually sleep? The programmed duration — unless the
  // accelerometer's INT1 line woke us early (motion wake); then measure it from
  // the system time, which the RTC timer keeps running through deep sleep.
  // Accumulated at the very top so the epoch estimate is updated even on an
  // early-exit wake (e.g. battery cutoff).
  const bool motionWake = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0);
  uint32_t sleptSecs = s_lastSleepSecs;
  if (motionWake && s_sleepEnterUs > 0) {
    const int64_t d = nowUs() - s_sleepEnterUs;
    if (d > 0 && d < (int64_t)s_lastSleepSecs * 1000000LL) {
      sleptSecs = (uint32_t)((d + 500000LL) / 1000000LL);
    }
  }
  s_elapsedEstSecs += sleptSecs;
  if (motionWake) s_motionHoldoff = MOTION_WAKE_HOLDOFF_WAKES;

  Serial.begin(115200);
  delay(200);
  Serial.printf("\n=== Sensor Node %s  Wake #%lu ===\n",
                FIRMWARE_VERSION, (unsigned long)s_wakeCount);

  // Why did the CPU start? A deep-sleep wake is normal; anything else
  // (BROWNOUT, PANIC, TASK_WDT, POWERON) is a real reboot worth recording —
  // it also explains the s_wakeCount/seq resets seen in the logs.
  esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("[Boot] reset reason: %s\n", resetReasonStr(resetReason));
  // Stage breadcrumb: what was the previous run doing when it reset? (RTC
  // NOINIT memory, so it survives a panic/watchdog reset.) Also re-arms the
  // crumb for this run. Used in the bootlog line and the crash summary.
  uint8_t prevStage = crashBootBegin(resetReason);
  // Motion wake: the LIS2DW12 INT1 line pulled the node out of deep sleep.
  if (motionWake) {
    Serial.printf("[Boot] Woken by tilt/motion interrupt after %lu s — motion wake "
                  "disarmed for %u wakes\n", (unsigned long)sleptSecs,
                  (unsigned)MOTION_WAKE_HOLDOFF_WAKES);
  }

  // ---- Boot-loop detection ----
  // Count this reboot in NVS (survives the brownout/panic that wipes RTC RAM)
  // and decide whether we're stuck in a wake-#1 crash loop. If so, s_safeMode
  // gates off every radio path below; the wake still SD-logs and then takes a
  // long recovery sleep. Cleared by markBootConfirmed() at deep sleep.
  checkBootLoop(resetReason);

  // ---- Debug / upload button ----
  // Checked before the watchdog is armed and before anything touches
  // SPI/I2C/WiFi, so holding it gives an uninterrupted, WDT-free window.
  checkDebugButton();

  // ---- Hardware watchdog ----
  const esp_task_wdt_config_t wdtCfg = {
    .timeout_ms     = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_reconfigure(&wdtCfg);
  esp_task_wdt_add(NULL);

  // ---- Wake classification ----
  // Decide up front whether this is a "watched" wake (boot, or a periodic
  // WiFi/OTA wake someone may be looking at). Frequent LoRa-only wakes are
  // unwatched, so we leave the OLED powered down to save its ~7–8 mA. These
  // same flags gate the WiFi/OTA work further down.
  // While WiFi keeps failing (no reachable network) stretch the interval to
  // WIFI_BACKOFF_EVERY_N — a failed attempt costs ~12 s of radio per SSID.
  // Wake #1 after any reset always tries (the streak lives in RTC RAM).
  const uint32_t wifiEvery = (s_wifiFailStreak >= WIFI_FAIL_BACKOFF_AFTER)
                             ? WIFI_BACKOFF_EVERY_N : WIFI_UPLOAD_EVERY_N;
  bool isWifiWake = (s_wakeCount == 1) || (s_wakeCount % wifiEvery == 0);
  // Deliberately NOT triggered on wake #1. A cold boot already runs the
  // highest-current work in the whole schedule (WiFi TX + radio), and stacking
  // an OTA check (another sustained network+flash burst) on top of it is what
  // pushes a marginal supply into a brownout. Because a brownout clears RTC
  // RAM, the wake counter resets to 1 and the node re-enters this exact
  // high-current path forever — a self-perpetuating reset loop that never
  // reaches a low-current normal wake. OTA still runs on its periodic schedule
  // and on demand via the Config sheet (see otaReqDue below — also never on
  // wake #1); it just no longer piles onto every cold boot.
  bool isOtaWake  = (s_wakeCount != 1) &&
                    (s_wakeCount % OTA_CHECK_EVERY_N   == 0);
  bool useDisplay = DISPLAY_ENABLED && (isWifiWake || isOtaWake);

  // ---- Display ----
  // displayInit() is the only thing that arms the panel; when skipped, all
  // other display*() calls below become safe no-ops (see display_mgr.cpp).
  if (useDisplay) {
    displayInit();
    displaySplash("SENSOR " FIRMWARE_VERSION, "Booting...");
  }

  // ---- SPI buses ----
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, -1);
  pinMode(PIN_SD_CS,    OUTPUT); digitalWrite(PIN_SD_CS,    HIGH);
  pinMode(PIN_LORA_NSS, OUTPUT); digitalWrite(PIN_LORA_NSS, HIGH);

  // ---- Sensors ----
  // Notify the Kalman filter about the actual previous sleep duration so
  // it can scale process noise Q correctly before the first readAllSensors().
  crashStageSet(STAGE_SENSORS_INIT);
  sensorsSetSleepDuration(sleptSecs);
  sensorsSetSurveyMode(s_surveyMode);   // long radar burst + QC tag when on
  sensorsSetMotionWake(motionWake);     // flags this reading moved=1 if INT1 woke us
  sensorsInit();                        // detects every connected distance sensor
  esp_task_wdt_reset();

  // Surface what was detected (no-op on the OLED if the panel is off). Every
  // present sensor is read each wake; the primary feeds the level pipeline.
  Serial.printf("[Sensor] Distance sensors: %s  (primary %s)\n",
                sensorsPresentName(), sensorTypeName(sensorsActiveType()));
  displaySplash("SENSOR " FIRMWARE_VERSION, sensorsPresentName());

  // ---- SD ----
  crashStageSet(STAGE_SD_INIT);
  sdInit();
  // A reset clears RTC RAM. Restore the recovery state with one SD scan; on
  // ordinary deep-sleep wakes the flag avoids repeatedly scanning the queue.
  if (s_wakeCount == 1) s_backlogPending = (pendingCount() > 0);

  // ---- Credentials ----
  // Load WiFi/GAS/LoRa-token from the SD card (cached in NVS) BEFORE the radio
  // or WiFi need them. Kept out of the binary so the OTA-published firmware.bin
  // carries no secrets. secretsLoad() is safe even if the SD didn't mount — it
  // falls back to the NVS cache, then to "no secrets" (skips WiFi, drops the
  // LoRa token) without faulting.
  crashStageSet(STAGE_SECRETS);
  secretsLoad();

  // ---- Early battery check ----
  // Read the cell before powering the radio so a critically low battery
  // skips the SX1262 init + its TX current entirely.  Only act on a
  // plausible reading (>2.5 V) so USB-only / no-battery operation isn't
  // mistaken for a low cell.  The authoritative cutoff (after the full
  // read + SD log) still runs below.
  float earlyBatV    = getBatteryVoltage();
  bool  batCritical  = (earlyBatV > BAT_PLAUSIBLE_MIN_V && earlyBatV < BAT_CUTOFF_V);

  // ---- LoRa ----
  bool loraOk = false;
  if (s_safeMode) {
    Serial.println(F("[SafeMode] Skipping LoRa init (radio off for recovery)"));
  } else if (batCritical) {
    Serial.printf("[WARN] Low battery %.2fV — skipping LoRa init\n", earlyBatV);
  } else {
    crashStageSet(STAGE_LORA_INIT);
    displaySplash("SENSOR " FIRMWARE_VERSION, "LoRa init...");
    loraOk = loraInit();
  }

  // ================================================================
  //  Read all sensors
  // ================================================================
  crashStageSet(STAGE_READ_SENSORS);
  displaySplash("SENSOR " FIRMWARE_VERSION, "Reading...");
  SensorData       data;
  SensorReadResult result = readAllSensors(data);
  esp_task_wdt_reset();

  // ----------------------------------------------------------------
  //  RTC epoch fallback
  //
  //  gmtime() here does raw calendar math with no timezone shift of its own —
  //  it just splits whatever epoch convention feeds it into Y/M/D H:M:S. Since
  //  data.epochSeconds is now a Malaysia-local (UTC+8) count (readRTC() ->
  //  rtc.now().unixtime(), and the RTC itself stores local time), this
  //  reconstruction is already correct without any further offset.
  // ----------------------------------------------------------------
  if (data.rtcValid) {
    s_lastKnownEpoch = data.epochSeconds;
    s_elapsedEstSecs = 0;
  } else if (s_lastKnownEpoch > 0) {
    uint32_t estEpoch = s_lastKnownEpoch + s_elapsedEstSecs;
    time_t   t        = (time_t)estEpoch;
    struct tm* tmInfo = gmtime(&t);
    if (tmInfo) {
      strftime(data.isoTimestamp, sizeof(data.isoTimestamp),
               "%Y-%m-%dT%H:%M:%S", tmInfo);
      data.epochSeconds = estEpoch;
      Serial.printf("[RTC] Estimated ts: %s (+%lu s)\n",
                    data.isoTimestamp, (unsigned long)s_elapsedEstSecs);
    }
  }

  // ---- Clock-trust guard (C2) ----
  // A readable RTC isn't necessarily a RIGHT one. Stop trusting its absolute
  // time — flag rtc_valid=0 so the cloud uses the gateway's clock (gw_ts)
  // instead of a wrong node_ts — when it was re-seeded from the firmware build
  // time after losing power and hasn't been synced since, when it reads
  // earlier than the last successful sync (it was reset), or when it hasn't
  // been NTP/gateway-corrected for longer than MAX_RTC_UNSYNCED_SEC (a
  // LoRa-only node with a drifting offset). See rtcTimeTrusted().
  if (data.rtcValid) {
    const char* why = nullptr;
    if (!rtcTimeTrusted(data.epochSeconds, &why)) {
      Serial.printf("[RTC] Clock not trusted (%s) — flagging rtc_valid=0; "
                    "cloud will use gw_ts\n", why ? why : "?");
      data.rtcValid = false;
    }
  }

  Serial.printf("[DATA] ts=%s  rtc=%s\n",
                data.isoTimestamp, data.rtcValid ? "OK" : "EST");
  if (result.distanceOk) {
    Serial.printf("       dist_raw=%.1f cm  dist_k=%.1f cm  water=%.1f cm  [%s]\n",
                  data.distanceRaw, data.distanceCm, data.waterLevelCm,
                  sensorTypeName(data.sensorType));
  } else {
    Serial.println(F("       dist=FAIL"));
  }
  Serial.printf("       radar=%.1f cm  ultrasonic=%.1f cm (sd %.2f, n %u)\n",
                data.distanceRadarCm, data.distanceUsCm,
                data.usBurstSd, (unsigned)data.usBurstN);
  if (result.envOk) {
    Serial.printf("       T=%.2f C  RH=%.1f%%  bat=%.2fV\n",
                  data.tempC, data.humidity, data.batV);
  }

  // ================================================================
  //  SD log — written every wake, before the battery cutoff check,
  //  so low-bat events are always captured for later analysis.
  // ================================================================
  crashStageSet(STAGE_SD_LOG);
  bool logged = sdLog(data, s_wakeCount, s_lastWifiRssi);
  if (!logged) Serial.println(F("[WARN] SD log failed"));

  // Record real reboots (not deep-sleep wakes) to /bootlog.csv with this wake's
  // timestamp, so brownouts/panics during unattended runs are visible later.
  // The reason carries the previous run's stage ("PANIC@LORA_TX") when known.
  if (resetReason != ESP_RST_DEEPSLEEP) {
    char reasonStage[40];
    if (prevStage != STAGE_NONE && prevStage != STAGE_SLEEP)
      snprintf(reasonStage, sizeof(reasonStage), "%s@%s", resetReasonStr(resetReason), crashStageName(prevStage));
    else
      snprintf(reasonStage, sizeof(reasonStage), "%s", resetReasonStr(resetReason));
    sdLogBootEvent(reasonStage, s_wakeCount, data.isoTimestamp);
  }
  // A core dump from a past panic waits in flash until a WiFi wake uploads it;
  // write its summary to the SD card now (once) so it survives even if WiFi
  // never comes — SD-first, like every other record on this node.
  crashLogToSdOnce(data.isoTimestamp, s_wakeCount);

  // ================================================================
  //  Battery cutoff
  //  When LiPo is critically low, skip LoRa/WiFi (both draw 100–120 mA
  //  peak) and sleep long to allow partial recovery. Uses SAFE_MODE_SLEEP_US
  //  (a genuine multi-minute sleep) — NOT SLEEP_MAX_US, which is pinned to the
  //  10 s bench cadence and would just re-wake into the cutoff every 10 s,
  //  never letting the cell (or solar) recover.
  // ================================================================
  if (data.batValid && data.batV < BAT_CUTOFF_V) {
    Serial.printf("[WARN] Low battery %.2fV — skipping TX, long recovery sleep\n", data.batV);
    // No radio this wake — queue the reading so it still reaches the cloud once
    // the cell recovers (a cheap SD append; tidelog.csv already holds it).
    pendingAppend(data);
    s_backlogPending = true;
    char batMsg[16];
    snprintf(batMsg, sizeof(batMsg), "%.2fV LOW", data.batV);
    displaySplash("LOW BATTERY", batMsg);
    delay(600);   // brief — every mJ counts when the cell is already critical
    enterDeepSleep(SAFE_MODE_SLEEP_US);
    // NOT REACHED
  }

  // ================================================================
  //  Sleep duration — fixed in survey mode, adaptive otherwise
  // ================================================================
  uint64_t sleepDurationUs = SLEEP_DURATION_US;

  if (s_surveyMode) {
    // Survey mode: a REGULAR time series is what tide reduction needs —
    // adaptive cadence would make interpolation at sounding epochs unreliable.
    sleepDurationUs = SURVEY_SLEEP_US;
    Serial.printf("[Sleep] Survey mode — fixed %llu s cadence\n",
                  sleepDurationUs / 1000000ULL);
  }
  // Use the RAW (pre-Kalman) distance for the wake-rate decision: sample
  // faster whenever the surface actually moves so the spike-rejecting filter
  // can confirm or reject it quickly — rather than reacting to the filter's
  // own smoothing transient (which previously caused phantom "fast change").
  else if (data.distanceValid && s_prevDistanceCm > 0.0f &&
           (uint8_t)data.sensorType == s_prevSensor) {   // compare a sensor with itself
    float delta = fabsf(data.distanceRaw - s_prevDistanceCm);
    if (delta > ADAPTIVE_DELTA_FAST_CM) {
      sleepDurationUs = SLEEP_MIN_US;
      Serial.printf("[Sleep] Fast change %.1f cm → %llu s\n",
                    delta, sleepDurationUs / 1000000ULL);
    } else if (delta < ADAPTIVE_DELTA_SLOW_CM) {
      sleepDurationUs = SLEEP_MAX_US;
      Serial.printf("[Sleep] Stable %.1f cm → %llu s\n",
                    delta, sleepDurationUs / 1000000ULL);
    }
  }
  if (data.distanceValid) {
    s_prevDistanceCm = data.distanceRaw;
    s_prevSensor     = (uint8_t)data.sensorType;
  }

  // ================================================================
  //  LoRa transmit (primary path — every wake)
  //
  //  Transmitted EVEN while an SD backlog is draining. An earlier design held
  //  LoRa TX back during backlog recovery so cloud rows stayed append-ordered,
  //  but that disabled LoRa for the whole (WiFi-paced) drain — so the moment
  //  the gateway recovered, the node couldn't tell, and everything stayed on
  //  the WiFi path. Ordering doesn't actually need it: every consumer (dashboard,
  //  tideSeries) sorts by the reading's own timestamp, not arrival order. So we
  //  keep transmitting: a fresh reading that gets ACKed reaches the cloud via
  //  the gateway immediately (and is NOT queued), while the old backlog keeps
  //  draining over WiFi in parallel. s_backlogPending now only orders the
  //  WiFi-upload path below (don't let a direct upload jump the queue).
  // ================================================================
  int8_t loraStatus = -1;

  if (s_safeMode) {
    Serial.println(F("[SafeMode] Skipping LoRa TX (radio off for recovery)"));
  } else if (!loraOk) {
    Serial.println(F("[LoRa] Skipping TX — init failed"));
  } else {
    // Transmit EVERY wake, even when the distance read failed (dist=-1). A radar
    // dropout must NOT silence the node — the payload still carries the rv/ev/bat
    // flags and the -1 sentinel, so the cloud logs a distance-flagged reading
    // ("node alive, sensor faulted") instead of going completely dark. This is
    // the root-cause fix for the sheet stalling on an 8-minute -1 run. (C1)
    if (!result.distanceOk)
      Serial.println(F("[LoRa] Distance invalid — TX anyway as a flagged reading"));
    crashStageSet(STAGE_LORA_TX);
    esp_task_wdt_reset();
    // While the gateway has been silent for a while, one attempt per wake is
    // enough to notice it coming back; full retries resume with the next ACK.
    const uint8_t attempts = (s_consecutiveNoAck >= LORA_NOACK_BACKOFF_AFTER)
                             ? LORA_BACKOFF_ATTEMPTS : LORA_MAX_RETRIES;
    if (attempts < LORA_MAX_RETRIES) {
      Serial.printf("[LoRa] Gateway silent for %lu wakes — %u attempt(s) this wake\n",
                    (unsigned long)s_consecutiveNoAck, (unsigned)attempts);
    }
    LoRaSendResult res = loraSend(data, s_lastWifiRssi, attempts);
    esp_task_wdt_reset();
    switch (res) {
      case LoRaSendResult::OK:     loraStatus = 1; break;
      case LoRaSendResult::NO_ACK: loraStatus = 2; break;
      default:                     loraStatus = 0; break;
    }
    Serial.printf("[LoRa] %s\n",
                  loraStatus == 1 ? "ACK" :
                  loraStatus == 2 ? "NO_ACK" : "FAIL");

    // Track how long the gateway has been unreachable. An ACK resets it; any
    // non-ACK (no reply, or TX error) advances it. A large, growing count is a
    // persistent gateway/RF outage (the node's own TX works — see the LoRa_OK
    // bench logs), worth flagging on-site. The cloud sees the same outage as a
    // stalled gateway heartbeat via the Sheets health-watch.
    if (loraStatus == 1) {
      s_consecutiveNoAck = 0;

      // Gateway -> node clock sync over LoRa: the ACK can carry the gateway's
      // current UTC+8 epoch. When it does, correct the RTC from it so the node
      // (and its SD log) keeps REAL time with NO WiFi/NTP of its own. It applies
      // from the next wake (this wake's SD row is already written) and refreshes
      // the last-sync record the C2 staleness guard checks.
      uint32_t gwEpoch = loraLastGatewayEpoch();
      if (gwEpoch > NTP_MIN_VALID_EPOCH) {
        rtcSyncIfDrifted(gwEpoch, RTC_NTP_MAX_SKEW_SEC);
        rtcMarkSynced(gwEpoch);
        Serial.printf("[RTC] Synced from gateway ACK (epoch %lu)\n",
                      (unsigned long)gwEpoch);
      }
    } else {
      s_consecutiveNoAck++;
      Serial.printf("[LoRa] Gateway unacknowledged for %lu consecutive wake(s)\n",
                    (unsigned long)s_consecutiveNoAck);
    }
  }

  // ================================================================
  //  WiFi / OTA / pending flush (secondary path — conditional)
  // ================================================================
  int8_t wifiStatus = -1;  // -1=not an upload wake / no data, 0=fail, 1=uploaded
  // True once a WiFi wake has (connected and) run the WiFi backlog flush this
  // wake — so the LoRa backlog flush below doesn't also drain the same queue.
  bool   wifiFlushed = false;
  // True once THIS reading is safe beyond the SD log: ACKed by the gateway or
  // uploaded directly. Anything still unsecured after the WiFi block is queued
  // in one place below — so no path (OTA-only wake, WiFi failure, safe mode)
  // can leave a reading out of the backlog. (C1) Flagged readings included.
  bool   readingSecured = (loraStatus == 1);

  if (s_safeMode) {
    Serial.println(F("[SafeMode] Skipping WiFi/OTA/flush (radio off for recovery)"));
  } else if (isWifiWake || isOtaWake) {
    crashStageSet(STAGE_WIFI_CONNECT);
    displaySplash("SENSOR " FIRMWARE_VERSION, "WiFi...");
    esp_task_wdt_reset();
    bool wifiOk = wifiConnect();
    esp_task_wdt_reset();

    if (wifiOk) {
      s_wifiFailStreak = 0;
      s_lastWifiRssi = wifiRSSI();

      // ---- NTP → DS3231 resync ----
      // The node's only clock is the DS3231. The whole system runs on Malaysia
      // local time (UTC+8), so the RTC is kept in LOCAL wall-clock. NTP is
      // fetched as plain UTC (offset 0 — unambiguous: time(nullptr) is always a
      // true Unix epoch, no reliance on ESP32's configTime-offset behaviour),
      // then RTC_TZ_OFFSET_SEC is added explicitly before writing the RTC. This
      // mirrors the gateway's formatIsoLocal() so node_ts and gw_ts share one
      // convention.
      crashStageSet(STAGE_NTP);
      configTime(0, 0, NTP_SERVER1, NTP_SERVER2);   // UTC epoch; local shift applied below
      uint32_t ntpStart = millis();
      time_t   nowUtc   = 0;
      while ((nowUtc = time(nullptr)) < (time_t)NTP_MIN_VALID_EPOCH &&
             millis() - ntpStart < NTP_SYNC_TIMEOUT_MS) {
        delay(100);
        esp_task_wdt_reset();
      }
      if (nowUtc >= (time_t)NTP_MIN_VALID_EPOCH) {
        uint32_t localEpoch = (uint32_t)nowUtc + RTC_TZ_OFFSET_SEC;
        rtcSyncIfDrifted(localEpoch, RTC_NTP_MAX_SKEW_SEC);
        // Record the sync (C2): lets the trust guard above tell "clock verified
        // recently" from "drifting for days" on later LoRa-only wakes, and
        // clears a post-power-loss "unverified" flag.
        rtcMarkSynced(localEpoch);
      } else {
        Serial.println(F("[RTC] NTP not ready — RTC not resynced this wake"));
      }
      esp_task_wdt_reset();

      // ---- Remote node config (Config sheet → survey / OTA / recovery) ----
      // One extra GET on a connection we already paid for. A fetch failure
      // just keeps the current settings — never flips them.
      bool otaRequested = false;
      {
        crashStageSet(STAGE_NODECFG);
        // Report the running firmware version (fw=) so the cloud can confirm an
        // OTA landed — the node→GAS nodecfg poll happens every WiFi wake and needs
        // no gateway involvement, so version visibility doesn't depend on LoRa.
        // &trd=1 confirms a tilt-baseline reset applied on an earlier wake, so
        // the cloud can clear that one-shot (it keeps offering it until then).
        char cfgQ[80];
        snprintf(cfgQ, sizeof(cfgQ), "action=nodecfg&id=%u&fw=%s%s",
                 (unsigned)NODE_ID, FIRMWARE_VERSION, s_tiltResetAck ? "&trd=1" : "");
        String cfgBody;
        if (gasFetch(cfgQ, cfgBody)) {
          s_tiltResetAck = false;   // any pending confirmation has now been delivered
          bool sm = (cfgBody.indexOf("\"sm\":1") >= 0);
          otaRequested = (cfgBody.indexOf("\"ota\":1") >= 0);
          if (sm != s_surveyMode) {
            s_surveyMode = sm;
            Serial.printf("[Config] Survey mode %s (from Config sheet) — "
                          "takes effect next wake\n", sm ? "ON" : "OFF");
          }
          // Remote recovery knobs — let an operator throttle a struggling field
          // node from the sheet, this same wake, without a reflash or a visit.
          s_pauseFlush = (cfgBody.indexOf("\"pf\":1") >= 0);
          s_flushCap   = 0;
          int fcIdx = cfgBody.indexOf("\"fc\":");
          if (fcIdx >= 0) s_flushCap = (uint32_t)atol(cfgBody.c_str() + fcIdx + 5);
          if (s_pauseFlush) Serial.println(F("[Config] pause_flush ON — backlog flush held"));
          if (s_flushCap)   Serial.printf("[Config] flush_cap = %lu this wake\n",
                                          (unsigned long)s_flushCap);
          // One-shot: the operator re-levelled the gauge — forget the tilt
          // baseline so the next read captures the new installed orientation,
          // and confirm it on the next poll (the cloud clears the flag then;
          // a repeat before that just re-captures the baseline).
          if (cfgBody.indexOf("\"tr\":1") >= 0) {
            tiltResetBaseline();
            s_tiltResetAck = true;
          }
        }
        esp_task_wdt_reset();
      }

      // ---- Crash report (H2) ----
      // If a panic left a core dump in flash, send its summary now (same
      // connection) and erase it. Before the first WiFi wake it already sits in
      // /crashlog.csv on the SD card.
      crashUploadIfAny(data.isoTimestamp);
      esp_task_wdt_reset();

      // OTA: the periodic schedule, or a Config request. A request is honoured
      // at most once per OTA_REQUEST_MIN_WAKES and never on wake #1 (see the
      // wake-classification note above) — a standing ota_update_requested=1
      // is then one GitHub check an hour, not one every WiFi wake. Either way
      // only with a healthy battery: a brownout mid-download wastes it.
      const bool otaReqDue = otaRequested && s_wakeCount != 1 &&
          (s_lastOtaReqWake == 0 || s_wakeCount - s_lastOtaReqWake >= OTA_REQUEST_MIN_WAKES);
      if (isOtaWake || otaReqDue) {
        if (!otaBatteryOk(earlyBatV)) {
          Serial.printf("[OTA] Battery %.2fV < %.2fV — update check deferred\n",
                        earlyBatV, OTA_MIN_BAT_V);
        } else {
          if (otaReqDue) s_lastOtaReqWake = s_wakeCount;
          crashStageSet(STAGE_OTA);
          displaySplash("SENSOR " FIRMWARE_VERSION, "OTA check...");
          otaInit();
          esp_task_wdt_reset();
          checkForOTAUpdate();
          esp_task_wdt_reset();
        }
      }

      if (isWifiWake) {
        // Always drain the backlog of previously-unconfirmed readings.
        // Preserve this state before flushing: even if this wake empties the
        // queue, the current reading must wait so it cannot overtake an older
        // record that was pending when the wake started.
        const bool backlogAtStart = s_backlogPending;
        // earlyBatV is the pre-radio resting voltage; pendingFlush() scales how
        // much of the backlog it drains this wake to it (and re-checks under
        // load) so a big queue can't brown out a depleted cell. s_flushCap is a
        // remote hard cap (0 = auto). pause_flush skips draining entirely.
        uint32_t flushed = 0;
        crashStageSet(STAGE_WIFI_FLUSH);
        if (s_pauseFlush) {
          Serial.println(F("[Pending] Flush paused by remote config"));
        } else {
          flushed = pendingFlush(earlyBatV, s_flushCap);
        }
        esp_task_wdt_reset();
        wifiFlushed = true;   // WiFi wake handled the backlog — don't LoRa-flush too
        s_backlogPending = (pendingCount() > 0);
        if (flushed > 0) {
          Serial.printf("[Pending] Recovered %lu queued readings\n",
                        (unsigned long)flushed);
        }
        // Only upload the CURRENT reading directly if LoRa didn't already
        // confirm delivery — otherwise the gateway is already forwarding it
        // and a direct upload would just double-hit GAS.
        if (!readingSecured) {   // (C1) back up even a distance-flagged reading
          crashStageSet(STAGE_UPLOAD);
          if (backlogAtStart || s_backlogPending) {
            // Queued below, behind the older backlog (keeps WiFi-path order).
            Serial.println(F("[Pending] Current reading held behind backlog"));
          } else if (uploadData(data, s_lastWifiRssi)) {
            wifiStatus     = 1;
            readingSecured = true;
          } else {
            wifiStatus = 0;      // direct upload failed — queued below
          }
          esp_task_wdt_reset();
        }
      }

      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);

    } else {
      Serial.println(F("[WiFi] Could not connect"));
      if (s_wifiFailStreak < 255) s_wifiFailStreak++;
      if (s_wifiFailStreak == WIFI_FAIL_BACKOFF_AFTER) {
        Serial.printf("[WiFi] %u failed WiFi wakes in a row — trying only every %lu wakes "
                      "until one succeeds\n", (unsigned)s_wifiFailStreak,
                      (unsigned long)WIFI_BACKOFF_EVERY_N);
      }
      if (isWifiWake && !readingSecured) wifiStatus = 0;
    }
  }

  // Anything the gateway didn't ACK and that wasn't uploaded directly waits in
  // the SD queue for the next flush (WiFi or LoRa). A LoRa-ACKed reading is not
  // queued — the gateway is already forwarding it, and queueing it would
  // double-upload and needlessly grow the queue. tidelog.csv holds it anyway.
  if (!readingSecured) {   // (C1) including a distance-flagged reading
    pendingAppend(data);
    s_backlogPending = true;
  }

  // ================================================================
  //  LoRa backlog flush (backlog-LoRa)
  //
  //  On wakes the WiFi flush did NOT handle — every non-WiFi wake, and a WiFi
  //  wake that couldn't connect — drain the pending queue over LoRa through the
  //  gateway, so a node with no WiFi still recovers its backlog. Runs only after
  //  the real-time reading was ACKed (loraStatus==1 => the link is proven up),
  //  respects the remote pause_flush knob, and is bounded/battery-gated inside
  //  loraFlushPending() (remote flush_cap included). earlyBatV is the
  //  pre-radio resting voltage.
  // ================================================================
  if (!s_safeMode && loraStatus == 1 && !wifiFlushed && !s_pauseFlush &&
      s_backlogPending) {
    esp_task_wdt_reset();
    crashStageSet(STAGE_LORA_FLUSH);
    uint32_t loraFlushed = loraFlushPending(earlyBatV, s_flushCap);
    esp_task_wdt_reset();
    if (loraFlushed > 0) {
      Serial.printf("[LoRaFlush] Recovered %lu queued readings over LoRa\n",
                    (unsigned long)loraFlushed);
    }
    s_backlogPending = (pendingCount() > 0);
  }

  // ================================================================
  //  OLED update
  // ================================================================
  const uint32_t wifiEveryNext = (s_wifiFailStreak >= WIFI_FAIL_BACKOFF_AFTER)
                                 ? WIFI_BACKOFF_EVERY_N : WIFI_UPLOAD_EVERY_N;
  uint32_t nextWifiIn = (s_wakeCount % wifiEveryNext == 0)
                        ? wifiEveryNext
                        : wifiEveryNext - (s_wakeCount % wifiEveryNext);

  crashStageSet(STAGE_DISPLAY);
  displayData(data, sdReady(), loraStatus, wifiStatus, s_wakeCount, nextWifiIn);
  // Hold long enough to read only when the panel is actually on (watched
  // wake); otherwise there's nothing to see, so keep the wake brief to save
  // battery.
  delay(useDisplay ? DISPLAY_HOLD_MS : DISPLAY_HOLD_SHORT_MS);

  // Safe mode: sleep long so the cell recovers and the wake counter (RTC RAM)
  // survives this CLEAN sleep, carrying the node out of the wake-#1 crash trap.
  // markBootConfirmed() (in enterDeepSleep) also clears the NVS counter, so the
  // node retries the normal path after this one recovery cycle.
  enterDeepSleep(s_safeMode ? SAFE_MODE_SLEEP_US
                            : cadenceSleepUs(sleepDurationUs));
  // NOT REACHED
}

// loop() is effectively never reached: setup() always ends in enterDeepSleep(),
// and a deep-sleep wake restarts the chip from setup() — not loop(). This is
// only a safety net that re-sleeps if setup() ever returned unexpectedly.
void loop() {
  esp_deep_sleep(SLEEP_DURATION_US);
}
