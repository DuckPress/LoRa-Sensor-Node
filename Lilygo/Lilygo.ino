/*
 * Lilygo T3-S3 v1.2 — Sensor Node Firmware  v1.2.0
 *
 * Sensors  : HLK-LD2413 24 GHz radar (UART: ESP RX=GPIO44, TX=GPIO43)
 *            SHT3x temp/humidity (I2C Wire1, SDA=GPIO16, SCL=GPIO15)
 *            DS3231 RTC          (I2C Wire1, shared bus)
 *
 * Storage  : Onboard MicroSD (SPI FSPI, default bus)
 * Display  : Onboard SSD1306 128×64 OLED (I2C Wire, SDA=18, SCL=17)
 *
 * Comms    : (1) LoRa SX1262 → XIAO ESP32-C3 + Wio-SX1262 gateway  [PRIMARY]
 *            (2) WiFi → Google Apps Script HTTPS                      [SECONDARY]
 *            (3) OTA via GitHub Releases                              [MAINTENANCE]
 *
 * Power    : Deep sleep between readings — 10–120 s adaptive.
 *            WiFi connects only every WIFI_UPLOAD_EVERY_N wakes.
 *            LiPo < BAT_CUTOFF_V skips radio TX entirely.
 *
 * Distance : 30 ultrasonic pulses → trim 8+8 → mean
 *            → temperature-corrected speed-of-sound
 *            → 1-D Kalman filter (Q scaled by actual sleep duration)
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

#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <esp_system.h>   // esp_reset_reason()
#include <Preferences.h>  // NVS boot-loop counter (survives brownout/panic)

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

// Set from the NVS boot-loop counter at the top of setup(): true means the node
// has rebooted without completing a wake too many times in a row and should run
// this wake in SAFE MODE (no radio, SD-log only, long recovery sleep).
static bool s_safeMode = false;

// Remote recovery knobs fetched from the Config sheet (?action=nodecfg) on WiFi
// wakes. pause_flush halts backlog draining; flush_cap (0 = auto/voltage-scaled)
// hard-limits entries per wake. Let an operator throttle a struggling field
// node from the sheet without a reflash or a site visit.
static bool     s_pauseFlush = false;
static uint32_t s_flushCap   = 0;

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
  shutdownPeripherals();
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
  // Accumulate elapsed time at the very top so the epoch estimate is
  // updated even on an early-exit wake (e.g. battery cutoff).
  s_wakeCount++;
  s_elapsedEstSecs += s_lastSleepSecs;

  Serial.begin(115200);
  delay(200);
  Serial.printf("\n=== Sensor Node %s  Wake #%lu ===\n",
                FIRMWARE_VERSION, (unsigned long)s_wakeCount);

  // Why did the CPU start? A deep-sleep wake is normal; anything else
  // (BROWNOUT, PANIC, TASK_WDT, POWERON) is a real reboot worth recording —
  // it also explains the s_wakeCount/seq resets seen in the logs.
  esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("[Boot] reset reason: %s\n", resetReasonStr(resetReason));

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
  bool isWifiWake = (s_wakeCount == 1) ||
                    (s_wakeCount % WIFI_UPLOAD_EVERY_N == 0);
  // Deliberately NOT triggered on wake #1. A cold boot already runs the
  // highest-current work in the whole schedule (WiFi TX + radio), and stacking
  // an OTA check (another sustained network+flash burst) on top of it is what
  // pushes a marginal supply into a brownout. Because a brownout clears RTC
  // RAM, the wake counter resets to 1 and the node re-enters this exact
  // high-current path forever — a self-perpetuating reset loop that never
  // reaches a low-current normal wake. OTA still runs on its periodic schedule
  // and on demand via the Config sheet (see otaRequested below); it just no
  // longer piles onto every cold boot.
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
  sensorsSetSleepDuration(s_lastSleepSecs);
  sensorsSetSurveyMode(s_surveyMode);   // long radar burst + QC tag when on
  sensorsInit();                        // auto-detects the connected sensor
  esp_task_wdt_reset();

  // Surface the auto-detected sensor (no-op on the OLED if the panel is off).
  Serial.printf("[Sensor] Active distance sensor: %s\n",
                sensorTypeName(sensorsActiveType()));
  displaySplash("SENSOR " FIRMWARE_VERSION, sensorTypeName(sensorsActiveType()));

  // ---- SD ----
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
    displaySplash("SENSOR " FIRMWARE_VERSION, "LoRa init...");
    loraOk = loraInit();
  }

  // ================================================================
  //  Read all sensors
  // ================================================================
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

  // ---- Clock-staleness guard (C2) ----
  // The RTC runs at the right rate but can hold a wrong OFFSET if it hasn't been
  // NTP-corrected in a long time (LoRa-only node, or a dead RTC backup cell). If
  // the gap since the last successful NTP sync exceeds MAX_RTC_UNSYNCED_SEC,
  // stop trusting the absolute time: flag rtc_valid=0 so the cloud uses the
  // gateway's clock (gw_ts) instead of a drifted node_ts. The rate is fine, so
  // (now - lastSync) is an honest elapsed time even when the offset is wrong.
  if (data.rtcValid) {
    Preferences clkPrefs;
    clkPrefs.begin("clk", true);                 // read-only
    uint32_t lastSync = clkPrefs.getUInt("lastsync", 0);
    clkPrefs.end();
    if (lastSync != 0 && data.epochSeconds > lastSync &&
        (data.epochSeconds - lastSync) > MAX_RTC_UNSYNCED_SEC) {
      Serial.printf("[RTC] Unsynced for ~%lu h (> %lu h) — flagging rtc_valid=0; "
                    "cloud will use gw_ts\n",
                    (unsigned long)((data.epochSeconds - lastSync) / 3600UL),
                    (unsigned long)(MAX_RTC_UNSYNCED_SEC / 3600UL));
      data.rtcValid = false;
    }
  }

  Serial.printf("[DATA] ts=%s  rtc=%s\n",
                data.isoTimestamp, data.rtcValid ? "OK" : "EST");
  if (result.distanceOk) {
    Serial.printf("       dist_raw=%.1f cm  dist_k=%.1f cm  water=%.1f cm\n",
                  data.distanceRaw, data.distanceCm, data.waterLevelCm);
  } else {
    Serial.println(F("       dist=FAIL"));
  }
  if (result.envOk) {
    Serial.printf("       T=%.2f C  RH=%.1f%%  bat=%.2fV\n",
                  data.tempC, data.humidity, data.batV);
  }

  // ================================================================
  //  SD log — written every wake, before the battery cutoff check,
  //  so low-bat events are always captured for later analysis.
  // ================================================================
  bool logged = sdLog(data, s_wakeCount, s_lastWifiRssi);
  if (!logged) Serial.println(F("[WARN] SD log failed"));

  // Record real reboots (not deep-sleep wakes) to /bootlog.csv with this wake's
  // timestamp, so brownouts/panics during unattended runs are visible later.
  if (resetReason != ESP_RST_DEEPSLEEP) {
    sdLogBootEvent(resetReasonStr(resetReason), s_wakeCount, data.isoTimestamp);
  }

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
  else if (data.distanceValid && s_prevDistanceCm > 0.0f) {
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
  if (data.distanceValid) s_prevDistanceCm = data.distanceRaw;

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
    esp_task_wdt_reset();
    LoRaSendResult res = loraSend(data, s_lastWifiRssi);
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
        Preferences clkPrefs;
        clkPrefs.begin("clk", false);
        clkPrefs.putUInt("lastsync", gwEpoch);
        clkPrefs.end();
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

  if (s_safeMode) {
    Serial.println(F("[SafeMode] Skipping WiFi/OTA/flush (radio off for recovery)"));
  } else if (isWifiWake || isOtaWake) {
    displaySplash("SENSOR " FIRMWARE_VERSION, "WiFi...");
    esp_task_wdt_reset();
    bool wifiOk = wifiConnect();
    esp_task_wdt_reset();

    if (wifiOk) {
      s_lastWifiRssi = wifiRSSI();

      // ---- NTP → DS3231 resync ----
      // The node's only clock is the DS3231. The whole system runs on Malaysia
      // local time (UTC+8), so the RTC is kept in LOCAL wall-clock. NTP is
      // fetched as plain UTC (offset 0 — unambiguous: time(nullptr) is always a
      // true Unix epoch, no reliance on ESP32's configTime-offset behaviour),
      // then RTC_TZ_OFFSET_SEC is added explicitly before writing the RTC. This
      // mirrors the gateway's formatIsoLocal() so node_ts and gw_ts share one
      // convention.
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
        // Record the sync time (C2): lets the staleness guard above tell "clock
        // verified recently" from "drifting for days" on later LoRa-only wakes.
        Preferences clkPrefs;
        clkPrefs.begin("clk", false);
        clkPrefs.putUInt("lastsync", localEpoch);
        clkPrefs.end();
      } else {
        Serial.println(F("[RTC] NTP not ready — RTC not resynced this wake"));
      }
      esp_task_wdt_reset();

      // ---- Remote node config (Config sheet → survey / OTA / recovery) ----
      // One extra GET on a connection we already paid for. A fetch failure
      // just keeps the current settings — never flips them.
      bool otaRequested = false;
      {
        // Report the running firmware version (fw=) so the cloud can confirm an
        // OTA landed — the node→GAS nodecfg poll happens every WiFi wake and needs
        // no gateway involvement, so version visibility doesn't depend on LoRa.
        char cfgQ[64];
        snprintf(cfgQ, sizeof(cfgQ), "action=nodecfg&id=%u&fw=%s",
                 (unsigned)NODE_ID, FIRMWARE_VERSION);
        String cfgBody;
        if (gasFetch(cfgQ, cfgBody)) {
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
        }
        esp_task_wdt_reset();
      }

      // A remote OTA request is serviced on this WiFi wake rather than waiting
      // for the periodic OTA schedule. A sleeping node cannot be woken over
      // the internet, so the response is bounded by the WiFi wake period.
      if (isOtaWake || otaRequested) {
        displaySplash("SENSOR " FIRMWARE_VERSION, "OTA check...");
        otaInit();
        esp_task_wdt_reset();
        checkForOTAUpdate();
        esp_task_wdt_reset();
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
        if (s_pauseFlush) {
          Serial.println(F("[Pending] Flush paused by remote config"));
        } else {
          flushed = pendingFlush(earlyBatV, s_flushCap);
        }
        esp_task_wdt_reset();
        s_backlogPending = (pendingCount() > 0);
        if (flushed > 0) {
          Serial.printf("[Pending] Recovered %lu queued readings\n",
                        (unsigned long)flushed);
        }
        // Only upload the CURRENT reading directly if LoRa didn't already
        // confirm delivery — otherwise the gateway is already forwarding it
        // and a direct upload would just double-hit GAS.
        if (loraStatus != 1) {   // (C1) back up even a distance-flagged reading
          if (backlogAtStart || s_backlogPending) {
            pendingAppend(data);
            s_backlogPending = true;
            Serial.println(F("[Pending] Current reading held behind backlog"));
          } else if (uploadData(data, s_lastWifiRssi)) {
            wifiStatus = 1;
          } else {
            // Direct upload failed too — queue for retry on the next wake.
            wifiStatus = 0;
            pendingAppend(data);
            s_backlogPending = true;
          }
          esp_task_wdt_reset();
        }
      }

      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);

    } else {
      Serial.println(F("[WiFi] Could not connect"));
      // Back up the current reading only if LoRa delivery wasn't confirmed.
      if (isWifiWake && loraStatus != 1) {   // (C1) queue even a flagged reading
        wifiStatus = 0;
        pendingAppend(data);
        s_backlogPending = true;
      }
    }

  } else {
    // Non-WiFi wake: buffer for the WiFi backup path ONLY if LoRa didn't
    // confirm delivery. A LoRa-ACKed reading is already reaching the cloud
    // via the gateway, so buffering it would double-upload and needlessly
    // grow the queue. tidelog.csv on SD remains the complete record.
    if (loraStatus != 1) {   // (C1) queue even a distance-flagged reading
      pendingAppend(data);
      s_backlogPending = true;
    }
  }

  // ================================================================
  //  OLED update
  // ================================================================
  uint32_t nextWifiIn = (s_wakeCount % WIFI_UPLOAD_EVERY_N == 0)
                        ? WIFI_UPLOAD_EVERY_N
                        : WIFI_UPLOAD_EVERY_N - (s_wakeCount % WIFI_UPLOAD_EVERY_N);

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
