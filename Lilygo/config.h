#pragma once
#include <Arduino.h>

// ================================================================
//  Firmware Version
// ================================================================
#define FIRMWARE_VERSION "1.2.2"

// ================================================================
//  Node Identity
//  Included in the LoRa payload ("id") so the gateway can track and
//  buffer multiple sensor nodes independently.  Give each node in a
//  deployment a unique value (1..7).
// ================================================================
constexpr uint8_t NODE_ID = 1;

// ================================================================
//  Distance Sensors — multi-driver build with runtime AUTO-DETECT
//
//  Enable EVERY driver you want available in the firmware (all three by
//  default). At boot the node probes the enabled drivers and uses whichever
//  sensor is actually connected — so you can swap sensors WITHOUT reflashing.
//  The detected type is cached in RTC RAM across deep sleep and re-checked
//  only on a cold boot or after SENSOR_REDETECT_FAILS consecutive failed
//  reads, so a swapped/failed sensor is picked up within a few wakes without
//  paying the probe cost on every wake.
//
//    ENABLE_SENSOR_RCWL1670  — RCWL-1670 ultrasonic (TRIG/ECHO)
//    ENABLE_SENSOR_LD2413    — HLK-LD2413 24 GHz radar (UART). Immune to air
//                              temperature/humidity, so the speed-of-sound
//                              correction is skipped for it at runtime.
//
//  HARDWARE NOTE: the LD2413 uses the UART header pins (GPIO43/44) and the
//  RCWL-1670 uses its own TRIG/ECHO pins (GPIO41/42), so the two are on
//  independent pins — both can be wired at the same time with no conflict.
//  Probe order is LD2413 (radar, preferred) → RCWL.
// ================================================================
#define ENABLE_SENSOR_LD2413
//#define ENABLE_SENSOR_RCWL1670   // disabled — radar-only build (RCWL ultrasonic unused)

#if !defined(ENABLE_SENSOR_RCWL1670) && !defined(ENABLE_SENSOR_LD2413)
  #error "config.h: enable at least one distance sensor (ENABLE_SENSOR_*)."
#endif

// If NO sensor is positively detected, read with this type anyway (best guess)
// while continuing to re-detect each wake until one confirms. MUST name an
// ENABLED sensor (SENSOR_TYPE_RCWL1670 / SENSOR_TYPE_LD2413).
#define SENSOR_DETECT_FALLBACK  SENSOR_TYPE_LD2413

// Consecutive invalid reads from the cached sensor that trigger a fresh
// auto-detect next wake (covers a sensor that was swapped, unplugged or died).
constexpr uint8_t SENSOR_REDETECT_FAILS = 3;

// ================================================================
//  Credentials — NOT compiled into the binary.
//
//  The node's firmware.bin is published for OTA over a PUBLIC repo, so it must
//  contain NO secrets (a compiled-in secrets.h would be recoverable with
//  `strings firmware.bin`). Instead the WiFi networks, GAS host/path and LoRa
//  token are read at boot from the SD-card file /secrets.txt into secret_store
//  (see secret_store.h + secrets.example.txt). Access them via the
//  secret*() accessors, never a compile-time constant.
// ================================================================
constexpr const char* SECRETS_FILENAME = "/secrets.txt";

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;
// Fast-reconnect: when the previous successful AP's channel + BSSID are cached
// in RTC RAM, WiFi.begin() can skip the all-channel scan. This bounds that
// targeted attempt before falling back to the full multi-SSID scan path.
constexpr uint32_t WIFI_FAST_TIMEOUT_MS    = 4000;

// HTTPS upload to GAS — bound the GET within the watchdog window.
constexpr uint32_t UPLOAD_HTTP_TIMEOUT_MS  = 15000;

// NTP time sync. The node has no live time source of its own — the DS3231 was
// only ever seeded from compile time — so its timestamps drift / sit in the
// wrong zone. On WiFi wakes the node fetches time from NTP offset by
// RTC_TZ_OFFSET_SEC and corrects the RTC if it has drifted more than
// RTC_NTP_MAX_SKEW_SEC.
//
// The RTC (and therefore every timestamp this node produces — tidelog.csv,
// the LoRa payload, the GAS upload) is kept in MALAYSIA LOCAL TIME (UTC+8),
// not UTC. The gateway's gw_ts and the Apps Script both follow the same +08:00
// convention, so timestamps are consistent end to end. Any OTHER consumer must
// read these as UTC+8, not UTC.
constexpr const char* NTP_SERVER1          = "pool.ntp.org";
constexpr const char* NTP_SERVER2          = "time.google.com";
constexpr uint32_t    NTP_SYNC_TIMEOUT_MS  = 5000;
constexpr int32_t     RTC_NTP_MAX_SKEW_SEC = 2;            // resync if off by > this
constexpr uint32_t    NTP_MIN_VALID_EPOCH  = 1700000000UL; // ~2023-11; "set" beyond this

// Malaysia = UTC+8. NTP is fetched as plain UTC (configTime(0,0,…) in
// Lilygo.ino, so time(nullptr) is unambiguously a true Unix epoch) and this
// offset is added EXPLICITLY before writing the DS3231, so the RTC holds local
// wall-clock without depending on ESP32's configTime-offset behaviour. The
// compile-time fallback seed (__DATE__/__TIME__, used only when the DS3231 lost
// power) needs NO further adjustment — it's already the build machine's local
// wall clock, so as long as that machine is set to Malaysia time it matches the
// RTC's local convention.
constexpr int32_t     RTC_TZ_OFFSET_SEC    = 8 * 3600;

// ================================================================
//  OTA Update — GitHub Releases
//
//  REQUIRED partition scheme:
//    Tools → Partition Scheme → Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
// ================================================================
#define OTA_GITHUB_HOST      "github.com"
#define OTA_GITHUB_USER      "DuckPress"           // GitHub account
#define OTA_GITHUB_REPO      "LoRa-Sensor-Node"    // public repo hosting the releases
#define OTA_VERSION_FILENAME "version.txt"
#define OTA_BIN_FILENAME     "firmware.bin"

constexpr uint32_t OTA_HTTP_TIMEOUT_MS = 15000;

// ================================================================
//  Deep Sleep & Wake Schedule
//
//  The device wakes, reads all sensors, transmits via LoRa, logs
//  to SD, optionally uploads via WiFi, updates the OLED, then goes
//  back to deep sleep.
//
//  SLEEP_DURATION_US  — how long to sleep between wakes (µs).
//  WIFI_UPLOAD_EVERY_N — do a WiFi cloud upload every N wakes.
//                         10 wakes × 30 s = ~5 min between uploads.
//  OTA_CHECK_EVERY_N  — check GitHub for firmware updates every N wakes.
//                         2880 wakes × 30 s = ~24 h.
// ================================================================
constexpr uint64_t SLEEP_DURATION_US   = 10ULL * 1000000ULL;  // 10 s — bench-test cadence
constexpr uint32_t WIFI_UPLOAD_EVERY_N = 30;        // approximately every five minutes at a 10 s cadence
// Automatic OTA checks remain infrequent. A remote request in the Config tab
// causes a check on the next WiFi wake instead.
constexpr uint32_t OTA_CHECK_EVERY_N   = 25920;     // approximately every three days at a 10 s cadence

// ----------------------------------------------------------------
//  Survey mode — bathymetric-reference operation
//
//  When the gauge serves as the tide reference for a sounding survey, the
//  time series must be REGULAR (adaptive sleep makes interpolation at
//  sounding epochs unreliable) and each reading should average a proper
//  wave window. Survey mode therefore:
//    • forces a FIXED wake cadence (SURVEY_SLEEP_US, adaptive sleep off),
//    • lengthens the LD2413 radar burst (SURVEY_LD2413_* below — ~10 s of
//      frames at the 160 ms report cycle, a real wave-averaging window),
//    • tags every reading with survey_mode=1 end-to-end for QC.
//
//  It is toggled REMOTELY: set `survey_mode` to 1 in the Google Sheet's
//  Config tab and the node picks it up on its next WiFi wake (~5 min) — no
//  reflash, no site visit. The flag persists in RTC RAM across deep sleep.
//  Battery note: survey mode roughly triples the awake window per wake; use
//  it for survey sessions, not as the permanent mode.
// ----------------------------------------------------------------
constexpr uint64_t SURVEY_SLEEP_US          = 30ULL * 1000000ULL;  // fixed 30 s cadence
constexpr uint8_t  SURVEY_LD2413_SAMPLE_N   = 60;    // ~9.6 s of frames @160 ms
constexpr uint8_t  SURVEY_LD2413_TRIM_N     = 15;    // drop 15 low + 15 high (avg middle 30)
constexpr uint32_t SURVEY_LD2413_BUDGET_MS  = 12000; // burst hard cap (WDT is 60 s)

// ================================================================
//  Watchdog
//  Must cover: sensor burst + LoRa retries + (WiFi + HTTPS upload).
//  WiFi wakes add ~12 s for connect + ~5 s for upload = ~17 s extra.
// ================================================================
constexpr uint32_t WDT_TIMEOUT_S = 60;

// ================================================================
//  LilyGO T3-S3 v1.2 — Hardware Pin Map
// ================================================================

// Onboard OLED (SSD1306 128×64) — internal I2C, not on header
constexpr uint8_t PIN_OLED_SDA = 18;
constexpr uint8_t PIN_OLED_SCL = 17;
constexpr uint8_t OLED_ADDR    = 0x3C;
constexpr uint8_t OLED_WIDTH   = 128;
constexpr uint8_t OLED_HEIGHT  =  64;

// External I2C bus (SHT3x + DS3231) — Wire1, header-exposed GPIOs
constexpr uint8_t PIN_SENSOR_SDA = 16;
constexpr uint8_t PIN_SENSOR_SCL = 15;

// MicroSD (default SPI / FSPI)
constexpr uint8_t PIN_SD_MOSI = 11;
constexpr uint8_t PIN_SD_MISO =  2;
constexpr uint8_t PIN_SD_SCK  = 14;
constexpr uint8_t PIN_SD_CS   = 13;

// LoRa SX1262 (onboard — dedicated HSPI bus, separate from SD)
// Verified against T3-S3 v1.2 pinout diagram.
constexpr uint8_t PIN_LORA_SCK  =  5;
constexpr uint8_t PIN_LORA_MISO =  3;
constexpr uint8_t PIN_LORA_MOSI =  6;
constexpr uint8_t PIN_LORA_NSS  =  7;
constexpr uint8_t PIN_LORA_DIO1 = 33;
constexpr uint8_t PIN_LORA_RST  =  8;
constexpr uint8_t PIN_LORA_BUSY = 34;

// Battery voltage divider (÷2 resistor divider on T3-S3)
constexpr uint8_t PIN_BAT_ADC = 1;

// ----------------------------------------------------------------
//  Debug / upload button
//
//  Deep sleep powers down the T3-S3's native USB CDC, so the serial port
//  only exists for the few seconds the node is awake each wake — easy to
//  lose the race against the IDE/esptool. Holding this button LOW at boot
//  skips the entire sensor/LoRa/SD/WiFi pipeline and idles instead of
//  sleeping, so USB stays enumerated indefinitely and there's no time
//  pressure to trigger an upload. See checkDebugButton() in Lilygo.ino.
//
//  Deliberately NOT the onboard BOOT button (GPIO0): GPIO0 is a strapping
//  pin sampled by the ROM bootloader on reset, and its behaviour across a
//  deep-sleep wake isn't something these files can confirm for this board
//  revision — reusing it risks the chip dropping into the real ROM
//  bootloader instead of reaching this code. GPIO4 is a normal, unused,
//  non-strapping pin instead. Wire a momentary pushbutton between it and
//  GND (INPUT_PULLUP, active LOW, no external resistor needed) — verify 4
//  is actually free on your board's silkscreen/pinout diagram first.
// ----------------------------------------------------------------
constexpr uint8_t  PIN_DEBUG_BTN         = 4;
constexpr uint32_t DEBUG_MODE_TIMEOUT_MS = 600000UL;  // 10 min safety cap, then resumes normally

// ================================================================
//  Distance Sensor Parameters
// ================================================================

// RCWL-1670 (TRIG/ECHO) — GPIO41 TRIG, GPIO42 ECHO
constexpr uint8_t  PIN_RCWL_TRIG            = 41;
constexpr uint8_t  PIN_RCWL_ECHO            = 42;
constexpr float    RCWL1670_MIN_CM          = 2.0f;
constexpr float    RCWL1670_MAX_CM          = 450.0f;
constexpr uint8_t  RCWL1670_SAMPLE_N        = 30;  // more pings → lower per-reading noise
constexpr uint8_t  RCWL1670_TRIM_N          =  8;  // discard 8 lowest + 8 highest (avg middle 14)
constexpr uint32_t RCWL1670_ECHO_TIMEOUT_US = 27000;
constexpr uint32_t RCWL1670_INTER_SAMPLE_MS = 60;

// ----------------------------------------------------------------
//  HLK-LD2413 — 24 GHz FMCW liquid-level radar on the header's UART pins
//  (GPIO43/44), independent of the RCWL's TRIG/ECHO pins.
//
//  It STREAMS a distance frame every LD2413_CFG_REPORT_MS once out of config
//  mode — no per-reading trigger needed (unlike the ultrasonic RCWL). Distance
//  is a true range (radar = speed of light), so readAllSensors() does NOT apply
//  the speed-of-sound correction to it.
//
//  Wire protocol (HLK-LD2413 manual): 115200 8N1, little-endian.
//    Data frame (module → MCU), 14 bytes:
//      F4 F3 F2 F1 | len(2) | distance_mm(float32) | F8 F7 F6 F5
//    Command frame (MCU → module):
//      FD FC FB FA | len(2) | cmd(2) | data | 04 03 02 01
//  Logic is 3.3 V TTL (module TX is safe for the ESP32-S3); supply 3.3–5 V.
//
//  MODULE SILKSCREEN PINS: RX, OT1, GND, 3V3 (per the manual's Table 4 /
//  Figure 4-1 USB-adapter wiring diagram). OT1 is the module's UART TX
//  (data out) — it carries the distance frames and must go to PIN_LD2413_RX
//  below. Do NOT use OT2 for this: several third-party guides claim OT2 is
//  TX, but on this module OT2 is a different output, not the UART line.
//  Wiring OT2 here gives silence, not garbage, which makes it a confusing
//  failure to debug.
// ----------------------------------------------------------------
constexpr uint8_t  PIN_LD2413_RX = 44;   // ESP RX  <- sensor OT1 (UART TX / data stream — NOT "OT2")
constexpr uint8_t  PIN_LD2413_TX = 43;   // ESP TX  -> sensor RX (config commands)
// Optional high-side power-enable (drives a P-MOSFET/load-switch gate). The
// radar draws ~90 mA continuously, so a battery node should cut it during deep
// sleep. -1 = sensor is always powered (no switch fitted).
constexpr int8_t   PIN_LD2413_PWR = -1;

constexpr uint32_t LD2413_BAUD             = 115200;
constexpr float    LD2413_MIN_CM           =   15.0f;  // module floor 0.15 m
constexpr float    LD2413_MAX_CM           = 1000.0f;  // module ceiling 10 m
constexpr uint8_t  LD2413_SAMPLE_N         =   20;     // frames collected per read
constexpr uint8_t  LD2413_TRIM_N           =    5;     // drop 5 low + 5 high
constexpr uint32_t LD2413_FRAME_TIMEOUT_MS =  300;     // per-frame read timeout
constexpr uint32_t LD2413_READ_BUDGET_MS   = 2500;     // hard cap on the whole burst
constexpr uint32_t LD2413_BOOT_MS          =  700;     // power-up → first data frame

// One-time module configuration. The min/max range + report cycle are stored
// in the LD2413's OWN flash, so this only needs to run ONCE per physical
// module. Keep 0 on a battery node (the ~1 s config handshake every wake is
// pure overhead); set to 1 for a single boot against a factory-fresh module,
// then set back to 0 and re-flash.
#define LD2413_CONFIGURE_ON_BOOT 0
constexpr uint16_t LD2413_CFG_MIN_MM    =  150;   // reporting floor (mm)
constexpr uint16_t LD2413_CFG_MAX_MM    = 6000;   // reporting ceiling (mm) — SENSOR_HEIGHT + margin
constexpr uint16_t LD2413_CFG_REPORT_MS =  160;   // stream cycle (ms) — 50..1000

// SHT3x I2C address
constexpr uint8_t SHT3X_ADDR  = 0x44;
constexpr uint8_t DS3231_ADDR = 0x68;   // RTC — used by the I2C bus health-check

// ----------------------------------------------------------------
//  Optional mount-tilt / movement monitoring (LIS2DW12 on Wire1)
//
//  For a survey REFERENCE gauge, a shifted or settled mount silently biases
//  every reduced depth. An ST LIS2DW12 3-axis accelerometer (I2C 0x19, shares
//  the SHT3x/DS3231 Wire1 bus — 0x19 avoids SHT3x 0x44 and DS3231 0x68)
//  measures the gauge's tilt; a change from the installed baseline beyond
//  TILT_MOVED_DEG raises the "moved" flag (mv=1), carried to the sheet/
//  dashboard and flagged by the tide-reduction QA. The baseline is captured on
//  the first read after a cold boot and held in RTC RAM.
//
//  OFF by default — enable only when a LIS2DW12 is fitted. NOTE: this driver
//  is UNVERIFIED on real hardware; bench-test before relying on it for QA.
// ----------------------------------------------------------------
//#define ENABLE_TILT_LIS2DW12
constexpr uint8_t LIS2DW12_ADDR  = 0x19;   // SA0/SDO high; use 0x18 if tied low
constexpr float   TILT_MOVED_DEG = 1.0f;   // tilt change (deg) that flags "moved"

// ================================================================
//  Kalman Filter — 1-D constant-position model
//
//  Applied to the trimmed-mean ultrasonic reading across wakes.
//  State persists in RTC RAM so the filter warms up across deep
//  sleep cycles rather than restarting cold every reading.
//
//  KALMAN_Q  — process noise variance (cm²).
//              How much the true distance is expected to change
//              between two 30-second readings.  Higher → trusts
//              new measurements more; lower → smoother but slower
//              to follow real changes.
//
//  KALMAN_R  — measurement noise variance (cm²).
//              Residual uncertainty of the trimmed-mean reading
//              after outlier removal.  Higher → trusts the model
//              more; lower → trusts measurements more.
//
//  KALMAN_P0 — initial error covariance.  Large value = "I don't
//              know where the water is yet."  Converges in a few
//              readings.
// ================================================================
constexpr float KALMAN_Q  =  0.5f;      // cm² — low: tide changes slowly, so trust
                                        //   the model and smooth out ±2 cm sensor
                                        //   jitter. The spike gate (below) handles
                                        //   outliers, so heavy smoothing no longer
                                        //   risks the old slow-recovery lag.
constexpr float KALMAN_R  =  9.0f;      // cm² — treat each reading as noisier →
                                        //   more smoothing of the steady signal.
                                        //   Fallback only: used when this wake's
                                        //   burst standard deviation is unknown.
                                        //   Otherwise the filter uses the ACTUAL
                                        //   burst_sd² for R (see kalmanUpdate) —
                                        //   this fixed value is just the estimate
                                        //   burst_sd itself now replaces per-wake.
constexpr float KALMAN_R_MIN = 1.0f;    // cm² — floor for that burst-derived R, so
                                        //   an unusually tight burst can't make the
                                        //   filter over-trust a single reading.
constexpr float KALMAN_P0 = 1000.0f;    // large → fast initial convergence

// ----------------------------------------------------------------
//  Spike / outlier gate (applied BEFORE the Kalman update)
//
//  A trimmed-mean reading that deviates more than SPIKE_REJECT_CM from the
//  current estimate is treated as a glitch (blind-zone / multipath) — the
//  filter HOLDS its prediction instead of being dragged off by it. If such
//  large deviations persist for SPIKE_MAX_STREAK consecutive reads, the change
//  is real (or the filter was seeded on a bad value), so it re-seeds to the
//  new measurement and resumes tracking. This prevents a single bad ping (e.g.
//  a 9 cm blind-zone reading) from corrupting the output for many cycles.
// ----------------------------------------------------------------
constexpr float   SPIKE_REJECT_CM  = 15.0f;
constexpr uint8_t SPIKE_MAX_STREAK = 3;

// ----------------------------------------------------------------
//  Physical rate-of-change gate (guards the spike-gate RE-SEED)
//
//  The spike gate above re-seeds after SPIKE_MAX_STREAK large deviations,
//  assuming a persistent shift is real. But a transient sensor glitch (a radar
//  reflection off a passing boat/bird, a lost lock) can hold a wrong-but-STABLE
//  value for a few reads — low burst_sd, so the roughness QC passes it, yet it
//  is physically impossible. (Field data: the radar briefly read 4.18 m instead
//  of 2.46 m, "moving" the water 1.7 m in 30 s, and it slipped through.) So
//  before re-seeding, check the implied rate: real tidal water moves well under
//  ~0.5 cm/s even at peak flow, so anything above MAX_RATE_CM_PER_S is not water
//  — hold the last good estimate instead of snapping to the glitch. A deviation
//  that STAYS impossible for RESEED_HARD_ACCEPT_STREAK reads is finally accepted,
//  so a genuine gauge re-levelling can't lock the filter out forever.
//    Lower MAX_RATE_CM_PER_S = stricter (catches more glitches, but a genuinely
//    fast real rise — e.g. a river flash flood — is delayed until the escape
//    valve fires). Raise it for sites with legitimately fast level changes.
// ----------------------------------------------------------------
constexpr float   MAX_RATE_CM_PER_S         = 1.0f;   // > this ⇒ not real water
constexpr uint8_t RESEED_HARD_ACCEPT_STREAK = 30;     // escape valve (reads)

// ================================================================
//  LoRa RF Parameters — AS923-1 (Malaysian ISM, 920–923 MHz)
//  Must match the gateway (XIAO ESP32C3 + Wio-SX1262) exactly.
// ================================================================
constexpr float   LORA_FREQUENCY = 923.2f;
constexpr float   LORA_BW        = 125.0f;
constexpr uint8_t LORA_SF        =    9;
constexpr uint8_t LORA_CR        =    5;
constexpr int8_t  LORA_TX_POWER  =   22;

constexpr uint32_t LORA_ACK_TIMEOUT_MS = 3000;
constexpr uint8_t  LORA_MAX_RETRIES    =    3;
constexpr uint32_t LORA_RETRY_DELAY_MS = 1500;
constexpr size_t   LORA_PAYLOAD_SIZE   =  256;  // bumped: payload now includes distanceRaw

// ================================================================
//  Tide / Water-Level Geometry
// ================================================================
constexpr float SENSOR_HEIGHT_CM = 450.0f;

// ================================================================
//  SD Logging
// ================================================================
constexpr const char* LOG_FILENAME     = "/tidelog.csv";
// Where sdInit() rolls an existing tidelog.csv whose header doesn't match the
// current LOG_HEADER (a firmware schema change), so files never mix schemas.
constexpr const char* LOG_ROLLED_FILENAME = "/tidelog_old.csv";
// Boot/reset diagnostics — one line per non-deep-sleep reboot (brownout/panic).
constexpr const char* BOOTLOG_FILENAME = "/bootlog.csv";
constexpr const char* LOG_HEADER   =
    "timestamp,rtc_valid,dist_raw_cm,dist_kalman_cm,water_level_cm,"
    "temp_c,humidity_pct,bat_v,wifi_rssi,wake_count,"
    "burst_sd_cm,burst_n,survey_mode,mount_moved\n";

// ================================================================
//  Adaptive Deep Sleep
//
//  The sleep duration is chosen each wake based on how fast the
//  water level changed since the previous wake:
//
//  |Δdist| > ADAPTIVE_DELTA_FAST_CM → SLEEP_MIN_US   (rapid change)
//  |Δdist| < ADAPTIVE_DELTA_SLOW_CM → SLEEP_MAX_US   (stable)
//  otherwise                         → SLEEP_DURATION_US (normal)
//
//  The epoch-estimate accumulator uses the actual sleep duration
//  chosen so RTC fallback timestamps remain accurate even when the
//  device slows down during a calm period.
// ================================================================
constexpr uint64_t SLEEP_MIN_US           = 10ULL  * 1000000ULL;  // 10 s  — fast
constexpr uint64_t SLEEP_MAX_US           = 10ULL * 1000000ULL;  // 10 s — pinned to the fixed cadence (adaptive off)
constexpr float    ADAPTIVE_DELTA_FAST_CM =  5.0f;  // cm — shorten sleep above this
constexpr float    ADAPTIVE_DELTA_SLOW_CM =  1.0f;  // cm — lengthen sleep below this

// ================================================================
//  Pending Upload Queue
//
//  On non-WiFi wakes the current reading is buffered to pending.csv.
//  On the next WiFi wake all buffered entries are uploaded to GAS
//  (best-effort) before the current reading, then the file is deleted.
//
//  PENDING_MAX_ENTRIES caps file growth when the gateway is offline
//  for a long period.  Once the limit is hit, new readings are dropped
//  from the queue (the backlog is kept).  No data is truly lost — the
//  full record is always available in tidelog.csv.
//
//  pending.csv columns (no header):
//    ts, dist_k, dist_raw, wl, temp, hum, bat, sd, n, sm, mv
//  (the trailing QC fields — burst std-dev, burst sample count, survey-mode
//   and mount-moved flags — are optional: lines queued by older firmware carry
//   only the first 7 and still parse, with sd=-1, n=0, sm=0, mv=0 defaults.)
// ================================================================
constexpr const char*  PENDING_FILENAME    = "/pending.csv";
// 7,500 records: approximately 31.25 h at a 15 s cadence, or 20.8 h at the
// current 10 s cadence. This uses roughly 600 KB of microSD space.
constexpr uint16_t     PENDING_MAX_ENTRIES = 7500;

//  PENDING_FLUSH_MAX_PER_WAKE bounds how many queued entries are uploaded
//  in a single WiFi wake.  Each upload is a full TLS GET to GAS (~1–3 s) +
//  a 300 ms settle delay, so this directly caps awake time / energy on a
//  flush wake.  Entries beyond the budget are kept (oldest-first) for the
//  next WiFi wake.  40 × ~2.5 s ≈ 100 s typical flush window.
// Bounded recovery batch: larger than the fresh-reading rate during a
// five-minute WiFi interval, but not an unbounded battery/server drain.
constexpr uint16_t     PENDING_FLUSH_MAX_PER_WAKE = 40;
// Eight readings per HTTPS request keeps the encoded URL well below common
// web-server limits while reducing a 40-reading recovery from 40 requests to 5.
constexpr uint8_t      PENDING_BATCH_ENTRIES       = 8;
constexpr uint16_t     PENDING_BATCH_MAX_BODY      = 1200;

//  PENDING_MAX_RETRIES — an entry that has failed this many upload attempts
//  is dropped rather than kept forever, so one persistently-rejected reading
//  (e.g. a GAS-side validation issue) can't occupy a flush-budget slot on
//  every WiFi wake indefinitely. tidelog.csv on SD remains the complete
//  record regardless.
constexpr uint8_t      PENDING_MAX_RETRIES = 10;

//  PENDING_MAX_LINE_LEN — a single queue line longer than this is treated as
//  corruption (a torn write / FAT damage can leave a record with no newline,
//  which readStringUntil() would otherwise pull into RAM without bound). When
//  seen, the queue file is quarantined to PENDING_BAD_FILENAME and a fresh one
//  started, so a corrupt queue can't OOM the node or jam the flush forever.
constexpr uint16_t     PENDING_MAX_LINE_LEN = 300;
constexpr const char*  PENDING_BAD_FILENAME = "/pending.bad";

// ----------------------------------------------------------------
//  Voltage-scaled backlog flush
//
//  Draining the pending queue is the highest sustained-current work the node
//  does — WiFi TX for many seconds, up to PENDING_FLUSH_MAX_PER_WAKE uploads.
//  On a depleted cell that burst can pull the rail into a BROWNOUT, and because
//  a brownout clears RTC RAM (resetting the wake counter to 1, which is always
//  a WiFi wake), the node drops into a cold-boot flush loop it can't escape and
//  the battery never recovers. So the per-wake flush budget is SCALED to the
//  pre-radio battery voltage: flush freely when healthy, a little when low,
//  nothing near cutoff. This is a self-limiting drain — as flushing depletes
//  the cell the budget shrinks toward zero on its own, letting solar recharge
//  between wakes. PENDING_FLUSH_MAX_PER_WAKE above is the top ("full") tier.
constexpr float    BAT_FLUSH_FULL_V   = 3.90f;  // >= this: full budget (MAX_PER_WAKE)
constexpr float    BAT_FLUSH_MED_V    = 3.70f;  // >= this: medium budget
constexpr float    BAT_FLUSH_LOW_V    = 3.50f;  // >= this: small budget; below: skip flush
constexpr uint16_t PENDING_FLUSH_MEDIUM = 15;
constexpr uint16_t PENDING_FLUSH_SMALL  =  5;

//  Loaded-voltage guard. The tier above uses the pre-radio RESTING voltage,
//  but brownout is caused by the LOADED voltage during WiFi TX — a cold, aged,
//  or high-internal-resistance cell can read fine at rest and still sag under
//  load. After the first upload burst this wake, the flush re-reads the cell
//  WHILE WiFi is still up; if it has sagged below this floor, it stops draining
//  and defers the rest so the node doesn't chase itself into a brownout that
//  the resting reading couldn't see.
constexpr float    BAT_FLUSH_SAG_FLOOR_V = 3.45f;

// ================================================================
//  OLED display hold time before sleep
//
//  The device stays awake this long after updating the display so a
//  human can read it. That awake time is pure overhead on a battery
//  node, so the full hold is only used on wakes someone is likely to
//  be watching (boot + the periodic WiFi/OTA wakes). The frequent
//  LoRa-only wakes use the much shorter DISPLAY_HOLD_SHORT_MS — on a
//  fast 10 s cycle the old flat 3 s hold was ~30 % of the duty cycle.
// ================================================================
constexpr uint32_t DISPLAY_HOLD_MS       = 3000;
constexpr uint32_t DISPLAY_HOLD_SHORT_MS =  400;

// Master switch for the onboard OLED. Set to 0 for a deployed/headless unit to
// skip OLED init + drawing entirely (saves its ~7–8 mA whenever awake). When 1,
// the panel is still only powered on "watched" wakes (boot + WiFi/OTA) — see
// Lilygo.ino — so unattended LoRa-only wakes draw nothing for the display.
#define DISPLAY_ENABLED 1

// ================================================================
//  Battery cutoff
//
//  When the LiPo drops below BAT_CUTOFF_V the node skips LoRa TX
//  and WiFi entirely (both can draw 100–120 mA peak) and goes
//  directly back to deep sleep at SLEEP_MAX_US to give the battery
//  time to recover.  The SD log still captures the low-bat event.
// ================================================================
constexpr float BAT_CUTOFF_V = 3.3f;

// Plausibility window for a LiPo reading. Outside this band the value is
// treated as "no real battery" (USB-only / no-cell operation) rather than a
// low cell, so the cutoff path isn't tripped by a floating/absent ADC. Shared
// by the early pre-radio check (Lilygo.ino) and the batValid flag (sensors.cpp).
constexpr float BAT_PLAUSIBLE_MIN_V = 2.5f;
constexpr float BAT_PLAUSIBLE_MAX_V = 4.5f;

// ================================================================
//  Boot-loop safe mode  (field self-recovery contingency)
//
//  A brownout or panic clears RTC RAM, so the wake counter resets to 1 and the
//  node re-enters its highest-current path (wake #1 = WiFi + flush + OTA). If
//  that path is what's crashing, the node loops there forever and the battery
//  never recovers — exactly the failure seen in bootlog.csv (every reboot at
//  wake_count=1). A counter in NVS/flash SURVIVES the reset the RTC-RAM counter
//  can't: each unconfirmed reboot increments it, and a wake that runs to
//  completion clears it. After SAFE_MODE_BOOT_THRESHOLD consecutive
//  unconfirmed reboots the node enters SAFE MODE for one cycle — it skips all
//  radio work (LoRa/WiFi/OTA/flush), still logs to SD, and deep-sleeps
//  SAFE_MODE_SLEEP_US so the cell recharges and RTC RAM (the wake counter)
//  survives the CLEAN sleep, breaking it out of the wake-#1 trap.
constexpr uint32_t SAFE_MODE_BOOT_THRESHOLD = 5;
constexpr uint64_t SAFE_MODE_SLEEP_US       = 300ULL * 1000000ULL;  // 5 min recovery

// ================================================================
//  Sensor Validity Thresholds
// ================================================================
constexpr float TEMP_MIN_C  = -20.0f;
constexpr float TEMP_MAX_C  =  80.0f;
constexpr float HUM_MIN_PCT =   0.0f;
constexpr float HUM_MAX_PCT = 100.0f;
