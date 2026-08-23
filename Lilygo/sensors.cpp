#include "sensors.h"
#include "config.h"
#include <Wire.h>
#include <SHT31.h>          // Rob Tillaart SHT31 library v0.5+
#include <RTClib.h>         // Adafruit RTClib
#include <esp_task_wdt.h>
#include <string.h>         // memcpy (LD2413 frame parse)

// ================================================================
//  Peripheral objects
//  External sensors use Wire1 (second I2C controller) on header
//  pins PIN_SENSOR_SDA/SCL (GPIO15/16).  Wire is reserved for the
//  onboard OLED (GPIO17/18) which is internal-only.
// ================================================================
static TwoWire    I2CSensors(1);
static SHT31      sht(SHT3X_ADDR, &I2CSensors);
static RTC_DS3231 rtc;

// UART peripheral for the LD2413 radar (GPIO43/44).
#if defined(ENABLE_SENSOR_LD2413)
static HardwareSerial UartSensor(1);
static bool           s_uartBegun = false;
static void uartEnd() { if (s_uartBegun) { UartSensor.end(); s_uartBegun = false; } }
#endif

// ================================================================
//  Auto-detected active sensor — persists across deep sleep in RTC RAM.
//  NONE (0) on a cold boot forces detection on wake #1; a confirmed type is
//  reused on later wakes; SENSOR_REDETECT_FAILS bad reads force a re-detect.
// ================================================================
RTC_DATA_ATTR static SensorType s_activeType    = SENSOR_TYPE_NONE;
RTC_DATA_ATTR static bool       s_typeConfirmed = false;   // positively detected?
RTC_DATA_ATTR static uint8_t    s_failStreak    = 0;       // consecutive invalid reads

const char* sensorTypeName(SensorType t) {
  switch (t) {
    case SENSOR_TYPE_RCWL1670: return "RCWL-1670";
    case SENSOR_TYPE_LD2413:   return "LD2413";
    default:                   return "NONE";
  }
}
SensorType sensorsActiveType() { return s_activeType; }

// Forward declarations — definitions are lower in the file, next to the
// per-sensor read code they depend on.
static void       sensorBegin(SensorType t);
static SensorType sensorsDetect();

// ================================================================
//  Kalman filter state — persists across deep sleep in RTC RAM.
// ================================================================
RTC_DATA_ATTR static float   s_kalmanX            = 0.0f;
RTC_DATA_ATTR static float   s_kalmanP            = KALMAN_P0;
RTC_DATA_ATTR static bool    s_kalmanInit         = false;
RTC_DATA_ATTR static uint8_t s_kalmanRejectStreak = 0;   // consecutive spike rejects

// Seconds elapsed since the Kalman filter last INCORPORATED a measurement.
// sensorsSetSleepDuration() adds each wake's sleep gap; the filter resets it
// only when it actually folds a reading in. This way wakes with an invalid
// measurement or a spike-held reading still grow the predicted uncertainty
// instead of being silently forgotten. RTC RAM so it survives deep sleep.
RTC_DATA_ATTR static uint32_t s_kalmanElapsedSecs = 0;

// Last valid air temperature & humidity — used for the speed-of-sound
// correction, and as a fallback when the SHT3x misses a wake.  Defaults are
// typical Malaysian coastal conditions (28 °C, 80 % RH).
RTC_DATA_ATTR static float s_lastTempC    = 28.0f;
RTC_DATA_ATTR static float s_lastHumidity = 80.0f;

// Survey mode — set by Lilygo.ino each wake (persisted there in RTC RAM).
// Selects the longer LD2413 burst and tags the reading for QC.
static bool s_surveyMode = false;

void sensorsSetSurveyMode(bool on) { s_surveyMode = on; }

// Burst QC stats filled by collapseBurst() during the last distance read.
static float   g_burstSd = -1.0f;
static uint8_t g_burstN  = 0;

// ================================================================
//  sensorsSetSleepDuration — tell the Kalman filter how long the PREVIOUS deep
//  sleep lasted (seconds) so it can scale its process noise Q (a long gap means
//  more could have changed). Call once per wake, before readAllSensors().
//  The value ACCUMULATES until the filter next incorporates a measurement, so
//  gaps spanning invalid/spike-held wakes are not lost.
// ================================================================
void sensorsSetSleepDuration(uint32_t secs) {
  s_kalmanElapsedSecs += (secs > 0) ? secs : 1;
}

// ================================================================
//  1-D Kalman filter with sleep-duration-scaled process noise.
//
//  Physical justification for the scaling:
//    Q represents the variance of how much the true distance can
//    change in one timestep.  If the timestep is dt seconds and the
//    nominal step is T₀ = 30 s, then Q(dt) = KALMAN_Q × dt/T₀.
//
//    Short sleep (10 s) → smaller per-step variance → smoother.
//    Long sleep  (120 s) → larger per-step variance → more responsive.
// ================================================================
// burstSd = this wake's burst standard deviation (cm), or -1 if unknown
// (collapseBurst() couldn't compute one — e.g. too few in-range samples).
// When known, it directly measures this specific reading's noise/roughness,
// so it replaces the fixed KALMAN_R estimate for the measurement-noise term:
// a calm burst is trusted more than KALMAN_R assumed, a rough one less.
static float kalmanUpdate(float z, float burstSd) {
  // z = this wake's measurement (the temp-corrected trimmed-mean distance).
  // NOMINAL_SLEEP_S = the cadence Q was tuned for (30 s); used to scale Q below.
  constexpr float NOMINAL_SLEEP_S = (float)(SLEEP_DURATION_US / 1000000ULL);

  // First reading ever (or just after a re-seed): there is no prior estimate, so
  // adopt the measurement as the state and start "very unsure" (KALMAN_P0).
  if (!s_kalmanInit) {
    s_kalmanX            = z;           // state estimate   = the measurement
    s_kalmanP            = KALMAN_P0;   // error covariance = large (unsure)
    s_kalmanInit         = true;
    s_kalmanRejectStreak = 0;
    s_kalmanElapsedSecs  = 0;           // measurement incorporated — reset gap
    return z;
  }

  // ── Spike gate ──────────────────────────────────────────────────────────
  // Reject a reading that jumps more than SPIKE_REJECT_CM from the estimate
  // (almost always a blind-zone / multipath glitch) so it can't corrupt the
  // filter. If the deviation PERSISTS, it's a real shift (or a bad initial
  // seed), so re-seed to the new value with high uncertainty and resume.
  if (fabsf(z - s_kalmanX) > SPIKE_REJECT_CM) {
    // Still within the allowed streak → treat as a one-off glitch: skip the
    // update entirely and return the last good estimate (one bad ping vanishes).
    if (++s_kalmanRejectStreak < SPIKE_MAX_STREAK) {
      Serial.printf("[Kalman] Spike %.1f cm vs est %.1f cm — held (%u/%u)\n",
                    z, s_kalmanX, s_kalmanRejectStreak, SPIKE_MAX_STREAK);
      // No measurement incorporated — leave s_kalmanElapsedSecs accumulating
      // so the next accepted reading gets the full-gap process noise.
      return s_kalmanX;                 // glitch — hold last estimate
    }
    // The jump has now repeated SPIKE_MAX_STREAK times. Before snapping to it,
    // apply the PHYSICAL rate-of-change gate: real water can't move this far
    // this fast. If the implied rate is impossible, this is a lingering sensor
    // glitch (low burst_sd but wrong), so keep HOLDING the last good estimate
    // rather than corrupting the filter with it — unless it has persisted long
    // enough (RESEED_HARD_ACCEPT_STREAK) to be a genuine re-level.
    float dtSecs  = (float)(s_kalmanElapsedSecs > 0 ? s_kalmanElapsedSecs : 1);
    float rateCmS = fabsf(z - s_kalmanX) / dtSecs;
    if (rateCmS > MAX_RATE_CM_PER_S &&
        s_kalmanRejectStreak < RESEED_HARD_ACCEPT_STREAK) {
      Serial.printf("[Kalman] Implausible rate %.2f cm/s (%.0f cm in %.0f s) — "
                    "holding, not re-seeding (%u/%u)\n",
                    rateCmS, z - s_kalmanX, dtSecs,
                    s_kalmanRejectStreak, RESEED_HARD_ACCEPT_STREAK);
      return s_kalmanX;                 // transient glitch — keep the last good value
    }
    // Plausible shift (or it persisted long enough to be real): snap to it and
    // reset uncertainty to re-converge fast.
    Serial.printf("[Kalman] Sustained shift to %.1f cm — re-seeding filter\n", z);
    s_kalmanX            = z;           // real change / bad seed → snap to it
    s_kalmanP            = KALMAN_P0;
    s_kalmanRejectStreak = 0;
    s_kalmanElapsedSecs  = 0;           // re-seed counts as incorporating z
    return s_kalmanX;
  }
  s_kalmanRejectStreak = 0;             // reading was in range → reset the streak

  // ── Predict ──
  // PREDICT: grow the uncertainty by the process noise Q (how much the true
  // level could have drifted since the last INCORPORATED reading — accumulated
  // across any skipped/held wakes in between). A 120 s gap admits ~4× the
  // drift of a 30 s gap → the filter trusts a new reading more after long gaps.
  float qScaled = KALMAN_Q * ((float)s_kalmanElapsedSecs / NOMINAL_SLEEP_S);
  s_kalmanElapsedSecs = 0;              // gap consumed by this update
  float pPred   = s_kalmanP + qScaled;  // predicted error covariance

  // ── Update ──
  // UPDATE: the Kalman gain K ∈ [0,1] balances model vs measurement — K→1 when
  // our estimate is uncertain (trust the reading), K→0 when the reading is noisy
  // (large R → trust the model). Nudge the estimate toward z by K × the
  // residual, then shrink the covariance now that a reading has been folded in.
  // R is this wake's ACTUAL burst variance when known (floored at KALMAN_R_MIN
  // so an unusually tight burst can't cause overtrust), else the fixed KALMAN_R.
  float rMeas = KALMAN_R;
  if (burstSd >= 0.0f) {
    float r = burstSd * burstSd;
    rMeas = (r > KALMAN_R_MIN) ? r : KALMAN_R_MIN;
  }
  float K   = pPred / (pPred + rMeas);             // Kalman gain
  s_kalmanX = s_kalmanX + K * (z - s_kalmanX);     // corrected estimate
  s_kalmanP = (1.0f - K) * pPred;                  // reduced uncertainty

  return s_kalmanX;
}

// ================================================================
//  Speed-of-sound compensation — temperature AND humidity
//
//  Models the speed of sound in MOIST air, then re-scales the raw distance
//  (which was computed with the fixed constant 0.0343 cm/µs ≈ 20 °C dry air).
//
//  Method (accurate, MCU-friendly; no barometer on this board so sea-level
//  pressure 1013.25 hPa is assumed — valid for a coastal/sea-level gauge):
//    1. Saturation vapour pressure p_sat(T)  — Tetens equation (hPa)
//    2. Actual vapour pressure   p_v = (RH/100)·p_sat
//    3. Water-vapour mole fraction  x_w = p_v / P   (P = 1013.25 hPa)
//    4. Dry-air speed   c_dry = 331.3·sqrt(1 + T/273.15)   (m/s)
//    5. Humid-air speed c = c_dry·(1 + 0.16·x_w)
//       (the 0.16 coefficient is calibrated to Cramer's data: +1.3 m/s at
//        20 °C / 100 % RH).
//
//  At 32 °C / 80 % RH the total correction is ≈ +2.7 % vs the 0.0343 baseline
//  (the humidity term contributes ≈ +0.6 % on top of temperature).
// ================================================================
static inline float speedOfSoundCmPerUs(float tempC, float humidity) {
  // 1–2: vapour pressure (hPa)
  float pSat = 6.1078f * powf(10.0f, (7.5f * tempC) / (tempC + 237.3f));
  float pV   = (humidity / 100.0f) * pSat;
  // 3: water-vapour mole fraction at sea-level pressure
  float xw   = pV / 1013.25f;
  // 4–5: dry-air speed (m/s) scaled by the humidity factor
  float cDry = 331.3f * sqrtf(1.0f + tempC / 273.15f);
  float c    = cDry * (1.0f + 0.16f * xw);
  return c / 10000.0f;   // m/s → cm/µs
}

static inline float tempCompFactor(float tempC, float humidity) {
  constexpr float ASSUMED = 0.0343f;   // speed used during raw RCWL sampling
  return speedOfSoundCmPerUs(tempC, humidity) / ASSUMED;
}

// ================================================================
//  Sort / filter helpers — turn a noisy burst of ultrasonic pings into one
//  robust distance value.
// ================================================================

// qsort() comparator for ascending float order (branchless: yields -1/0/+1).
static int cmpFloat(const void* a, const void* b) {
  float fa = *(const float*)a, fb = *(const float*)b;
  return (fa > fb) - (fa < fb);
}

// Median of n samples (sorts the array in place). Used as a fallback when too
// few samples survived for a meaningful trimmed mean.
static float medianOf(float* arr, uint8_t n) {
  qsort(arr, n, sizeof(float), cmpFloat);
  return (n & 1) ? arr[n / 2] : (arr[n/2-1] + arr[n/2]) * 0.5f;
}

// Trimmed mean: sort, drop the trimN lowest AND trimN highest samples, then
// average the middle. Rejects the occasional wild ping (blind-zone/multipath)
// far better than a plain mean. Falls back to a full mean if trimming would
// remove every sample.
static float trimmedMean(float* arr, uint8_t n, uint8_t trimN) {
  qsort(arr, n, sizeof(float), cmpFloat);
  if (2u * trimN >= n) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < n; i++) sum += arr[i];
    return sum / n;
  }
  float   sum   = 0.0f;
  uint8_t count = n - 2u * trimN;
  for (uint8_t i = trimN; i < n - trimN; i++) sum += arr[i];
  return sum / count;
}

// Collapse a burst into one robust value AND record its QC statistics in
// g_burstSd / g_burstN. The standard deviation is computed over ALL in-range
// samples BEFORE trimming: that spread reflects the actual wave/noise state
// of the surface, which is exactly what a survey-grade uncertainty flag needs
// (the trimmed mean deliberately hides it). Shared by every distance sensor.
static float collapseBurst(float* arr, uint8_t n, uint8_t trimN) {
  g_burstN  = n;
  g_burstSd = -1.0f;                    // unknown until proven otherwise
  if (n == 0) return -1.0f;             // nothing usable

  if (n >= 2) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < n; i++) sum += arr[i];
    const float mean = sum / n;
    float ss = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
      const float d = arr[i] - mean;
      ss += d * d;
    }
    g_burstSd = sqrtf(ss / (n - 1));    // sample standard deviation
  }
  // n == 1: sd stays -1 (a single sample carries no spread information).

  if (n < (uint8_t)(2u * trimN + 1u))   // too few to trim → median fallback
    return medianOf(arr, n);
  return trimmedMean(arr, n, trimN);    // normal: trimmed mean
}

// ================================================================
//  I2C bus health-check + recovery (Wire1: SHT3x 0x44 + DS3231 0x68)
//
//  The SHT3x and DS3231 share this bus, so when wiring/power glitches they
//  drop out together and a single stuck slave can wedge the whole bus. A
//  per-wake check logs which devices answered; if BOTH are silent we clock the
//  bus to release a stuck slave and re-init — recovering without a full reboot.
// ================================================================
static bool i2cPing(uint8_t addr) {
  I2CSensors.beginTransmission(addr);
  return (I2CSensors.endTransmission() == 0);
}

static void i2cBusRecover() {
  // A slave caught mid-byte by a power glitch can hold SDA LOW indefinitely,
  // wedging the whole bus so nobody can talk. The standard fix: take the lines
  // over as plain GPIOs, manually pulse the clock until the slave lets SDA go,
  // then issue a STOP to resynchronise every device on the bus.
  I2CSensors.end();                      // release the hardware I2C controller
  // I2C lines are open-drain: drive them open-drain here too, so a slave that
  // is still clock-stretching (holding SCL low) or holding SDA never fights a
  // push-pull HIGH from the MCU — the bus pull-ups provide the HIGH level.
  pinMode(PIN_SENSOR_SCL, OUTPUT_OPEN_DRAIN);  // we now clock the bus by hand
  digitalWrite(PIN_SENSOR_SCL, HIGH);
  pinMode(PIN_SENSOR_SDA, INPUT_PULLUP); // watch SDA; pull-up lets the slave release it
  // Clock out up to 9 pulses to free a slave holding SDA low.
  for (uint8_t i = 0; i < 9; i++) {      // 9 = a full data byte + the ACK bit
    digitalWrite(PIN_SENSOR_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(PIN_SENSOR_SCL, LOW);  delayMicroseconds(5);
  }
  // Issue a STOP: SDA low->high while SCL is high.
  pinMode(PIN_SENSOR_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SENSOR_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(PIN_SENSOR_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_SENSOR_SDA, HIGH); delayMicroseconds(5);   // the rising STOP edge
  I2CSensors.begin(PIN_SENSOR_SDA, PIN_SENSOR_SCL);           // hand the pins back to hardware
  delay(20);
}

static void i2cBusHealthCheck() {
  bool shtOk = i2cPing(SHT3X_ADDR);
  bool rtcOk = i2cPing(DS3231_ADDR);
  Serial.printf("[I2C] bus check: SHT3x(0x44)=%s  DS3231(0x68)=%s\n",
                shtOk ? "OK" : "--", rtcOk ? "OK" : "--");
  if (!shtOk && !rtcOk) {
    Serial.println(F("[I2C] both devices silent — attempting bus recovery"));
    i2cBusRecover();
    shtOk = i2cPing(SHT3X_ADDR);
    rtcOk = i2cPing(DS3231_ADDR);
    Serial.printf("[I2C] after recovery: SHT3x=%s  DS3231=%s\n",
                  shtOk ? "OK" : "--", rtcOk ? "OK" : "--");
  }
}

// ================================================================
//  sensorsInit
// ================================================================
void sensorsInit() {
  I2CSensors.begin(PIN_SENSOR_SDA, PIN_SENSOR_SCL);
  delay(50);

  // Check (and if needed recover) the shared sensor bus before touching the
  // devices, so a wedged bus from a previous wake can't blind both sensors.
  i2cBusHealthCheck();

  // --- Distance sensor: auto-detect on cold boot / after repeated read
  //     failures; otherwise just bring up the cached one (no probe cost each
  //     wake). sensorsDetect() leaves the detected sensor initialised. ---
  if (!s_typeConfirmed) {
    s_activeType = sensorsDetect();
    if (s_activeType != SENSOR_TYPE_NONE) {
      s_typeConfirmed = true;               // detected sensor is already begun
    } else {
      // Nothing answered — read with the fallback this wake so we still try to
      // produce data, and keep re-detecting (stays unconfirmed) until one
      // confirms (a valid read in readAllSensors promotes it).
      s_activeType = SENSOR_DETECT_FALLBACK;
      sensorBegin(s_activeType);
      Serial.printf("[Sensor] Falling back to %s (unconfirmed)\n",
                    sensorTypeName(s_activeType));
    }
  } else {
    sensorBegin(s_activeType);              // cached — just bring it up
    Serial.printf("[Sensor] Using cached sensor: %s\n",
                  sensorTypeName(s_activeType));
  }

  sht.begin();
  delay(15);
  if (!sht.isConnected()) {
    Serial.println(F("[Sensor] WARNING: SHT3x not found at 0x44"));
  } else {
    Serial.println(F("[Sensor] SHT3x OK"));
  }

  if (!rtc.begin(&I2CSensors)) {
    Serial.println(F("[Sensor] WARNING: DS3231 not found"));
  } else {
    if (rtc.lostPower()) {
      // The RTC now stores MALAYSIA LOCAL TIME (UTC+8) directly — see
      // RTC_TZ_OFFSET_SEC in config.h. __DATE__/__TIME__ is already the build
      // machine's LOCAL wall clock, so as long as that machine is set to
      // Malaysia time, it lines up with the RTC's convention with no further
      // offset — seed it as-is rather than shifting by a build/RTC timezone
      // delta that no longer exists between the two.
      Serial.println(F("[RTC] Lost power — seeding with compile time (local)"));
      DateTime compileLocal(F(__DATE__), F(__TIME__));
      rtc.adjust(compileLocal);
    }
    Serial.println(F("[Sensor] DS3231 OK"));
  }

  Serial.println(F("[Sensor] BAT ADC ready"));
}

// ================================================================
//  rtcSyncIfDrifted — correct the DS3231 from a trusted epoch (NTP).
//  The DS3231 is kept in MALAYSIA LOCAL time (UTC+8), so the caller passes a
//  LOCAL epoch (NTP UTC + RTC_TZ_OFFSET_SEC — see Lilygo.ino). RTClib's
//  DateTime(uint32_t)/unixtime() both treat the value as raw seconds-since-1970,
//  so the comparison and adjust stay in the same (local) base.
// ================================================================
bool rtcSyncIfDrifted(uint32_t localEpoch, int32_t maxSkewSec) {
  DateTime now = rtc.now();
  if (!now.isValid()) {
    rtc.adjust(DateTime(localEpoch));
    Serial.println(F("[RTC] was invalid — set from NTP"));
    return true;
  }
  int32_t skew = (int32_t)((int64_t)localEpoch - (int64_t)now.unixtime());
  if (skew > maxSkewSec || skew < -maxSkewSec) {
    rtc.adjust(DateTime(localEpoch));
    Serial.printf("[RTC] drift %ld s — resynced from NTP\n", (long)skew);
    return true;
  }
  Serial.printf("[RTC] in sync with NTP (skew %ld s)\n", (long)skew);
  return false;
}

// ================================================================
//  sensorsShutdown — release I2C bus before deep sleep
// ================================================================
void sensorsShutdown() {
#if defined(ENABLE_SENSOR_LD2413)
  uartEnd();          // stop the radar UART
  // Cut the radar's ~90 mA supply (if a load-switch is fitted) before sleep.
  if (PIN_LD2413_PWR >= 0) digitalWrite(PIN_LD2413_PWR, LOW);
#endif
  I2CSensors.end();   // Wire1: sensor bus (GPIO15/16)
  // Wire (OLED bus, GPIO17/18) is ended by the caller in Lilygo.ino
  // after the display has finished its last update.
}

// ================================================================
//  RCWL-1670 (TRIG/ECHO)
//  Takes RCWL1670_SAMPLE_N measurements, WDT-resets every 5 pulses,
//  returns the trimmed mean using constant 0.0343 cm/µs.
//  Temperature correction is applied in readAllSensors().
// ================================================================
#if defined(ENABLE_SENSOR_RCWL1670)
static float readRCWL1670Raw() {
  float   samples[RCWL1670_SAMPLE_N];   // in-range distances collected this burst
  uint8_t validCount = 0;               // how many of the pings were usable

  for (uint8_t i = 0; i < RCWL1670_SAMPLE_N; i++) {
    if (i % 5 == 0) esp_task_wdt_reset();   // feed the WDT every 5 pings (burst is slow)

    // Trigger one ping: a clean LOW, then a 10 µs HIGH pulse on TRIG tells the
    // sensor to emit its ultrasonic burst.
    digitalWrite(PIN_RCWL_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_RCWL_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_RCWL_TRIG, LOW);

    // ECHO stays HIGH for the round-trip flight time; pulseIn() returns that
    // width in µs, or 0 if no echo arrives within the timeout (= out of range).
    unsigned long durationUs = pulseIn(PIN_RCWL_ECHO, HIGH, RCWL1670_ECHO_TIMEOUT_US);

    if (durationUs == 0) { delay(RCWL1670_INTER_SAMPLE_MS); continue; }   // miss → skip

    // distance = time × speed-of-sound ÷ 2 (the pulse travels there AND back).
    // 0.0343 cm/µs is the ~20 °C dry-air value; readAllSensors() rescales it for
    // the measured temperature + humidity afterwards.
    float cm = durationUs * 0.0343f / 2.0f;
    if (cm >= RCWL1670_MIN_CM && cm <= RCWL1670_MAX_CM) samples[validCount++] = cm;
    delay(RCWL1670_INTER_SAMPLE_MS);    // settle gap so the next echo doesn't overlap
  }

  // Collapse the burst into one robust number (also records QC stats):
  return collapseBurst(samples, validCount, RCWL1670_TRIM_N);
}
#endif

// ================================================================
//  HLK-LD2413 (24 GHz liquid-level radar, UART)
//
//  The module STREAMS a distance frame every report cycle once out of config
//  mode, so a read just collects LD2413_SAMPLE_N fresh frames and takes the
//  same trimmed-mean/median as the ultrasonics. Distance is returned in cm
//  (converted from the frame's float millimetres); NO speed-of-sound
//  correction is applied (radar range is unaffected by air temp/humidity).
// ================================================================
#if defined(ENABLE_SENSOR_LD2413)

// Read one 14-byte data frame, blocking up to timeoutMs. On success writes the
// distance (mm) to mmOut and returns true. Syncs to the F4 F3 F2 F1 header
// byte-by-byte, then reads the 10-byte remainder and validates the tail.
static bool ld2413ReadFrame(float& mmOut, uint32_t timeoutMs) {
  static const uint8_t HDR[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
  uint32_t start    = millis();
  uint8_t  hdrMatch = 0;

  while (millis() - start < timeoutMs) {
    if (!UartSensor.available()) { delay(1); continue; }
    uint8_t b = (uint8_t)UartSensor.read();

    if (b == HDR[hdrMatch]) {
      if (++hdrMatch < 4) continue;
      hdrMatch = 0;

      // Header matched — pull the remaining 10 bytes:
      //   [0..1] length (LE)  [2..5] distance float (LE)  [6..9] tail
      uint8_t rest[10];
      uint8_t got = 0;
      while (got < 10 && millis() - start < timeoutMs) {
        if (UartSensor.available()) rest[got++] = (uint8_t)UartSensor.read();
        else delay(1);
      }
      if (got < 10) return false;

      // Validate the F8 F7 F6 F5 tail; a false header inside the payload just
      // fails here and the outer loop resynchronises on the next byte.
      if (rest[6] == 0xF8 && rest[7] == 0xF7 && rest[8] == 0xF6 && rest[9] == 0xF5) {
        float mm;
        memcpy(&mm, &rest[2], sizeof(mm));   // 4-byte LE float, matches ESP32 layout
        mmOut = mm;
        return true;
      }
      // tail mismatch — keep scanning within the timeout
    } else {
      // Restart the match; the header bytes are all distinct, so the only way
      // to already be mid-match is if this byte is itself HDR[0].
      hdrMatch = (b == HDR[0]) ? 1 : 0;
    }
  }
  return false;
}

static float readLD2413Raw() {
  // Drop any frames that streamed in while we were busy, so we read fresh ones.
  while (UartSensor.available()) UartSensor.read();

  // Survey mode uses the longer burst (a proper wave-averaging window for
  // tide-reduction data); normal mode keeps the short battery-friendly one.
  const uint8_t  nWant  = s_surveyMode ? SURVEY_LD2413_SAMPLE_N  : LD2413_SAMPLE_N;
  const uint8_t  trimN  = s_surveyMode ? SURVEY_LD2413_TRIM_N    : LD2413_TRIM_N;
  const uint32_t budget = s_surveyMode ? SURVEY_LD2413_BUDGET_MS : LD2413_READ_BUDGET_MS;

  static_assert(SURVEY_LD2413_SAMPLE_N >= LD2413_SAMPLE_N,
                "samples[] is sized by the survey burst");
  float   samples[SURVEY_LD2413_SAMPLE_N];
  uint8_t validCount = 0;
  uint32_t burstStart = millis();

  for (uint8_t i = 0; i < nWant; i++) {
    if (i % 5 == 0) esp_task_wdt_reset();   // feed the WDT during the burst

    float mm;
    if (ld2413ReadFrame(mm, LD2413_FRAME_TIMEOUT_MS)) {
      float cm = mm / 10.0f;                // frame is millimetres
      if (cm >= LD2413_MIN_CM && cm <= LD2413_MAX_CM) samples[validCount++] = cm;
    }
    // No inter-sample delay: the module paces itself at its report cycle.
    if (millis() - burstStart > budget) break;   // safety cap
  }

  // Collapse the burst into one robust number (also records QC stats):
  return collapseBurst(samples, validCount, trimN);
}

#if LD2413_CONFIGURE_ON_BOOT
// Build and send one command frame, then drain the module's ACK. We don't
// hard-validate the ACK — a failed set just leaves the module's previous
// (flash-stored) value, and the range gate in readLD2413Raw() still protects
// the reading.
static void ld2413SendCommand(uint16_t cmd, const uint8_t* data, uint16_t dataLen) {
  uint8_t buf[16];
  uint16_t payload = 2 + dataLen;               // command word + data
  buf[0] = 0xFD; buf[1] = 0xFC; buf[2] = 0xFB; buf[3] = 0xFA;
  buf[4] = payload & 0xFF; buf[5] = payload >> 8;
  buf[6] = cmd & 0xFF;     buf[7] = cmd >> 8;
  for (uint16_t i = 0; i < dataLen; i++) buf[8 + i] = data[i];
  uint8_t* tail = buf + 8 + dataLen;
  tail[0] = 0x04; tail[1] = 0x03; tail[2] = 0x02; tail[3] = 0x01;

  while (UartSensor.available()) UartSensor.read();   // clear stream/stale ACK
  UartSensor.write(buf, 8 + dataLen + 4);
  UartSensor.flush();                                    // wait for TX to drain
  delay(100);                                               // let the module act/ACK
  while (UartSensor.available()) UartSensor.read();   // discard the ACK
}

// One-time provisioning: min/max reporting range + stream cycle. These persist
// in the module's own flash, so this is gated behind LD2413_CONFIGURE_ON_BOOT.
static void ld2413Configure() {
  Serial.println(F("[LD2413] Applying one-time config (range + report cycle)..."));
  const uint8_t enter[2] = { 0x01, 0x00 };
  ld2413SendCommand(0x00FF, enter, 2);                      // enter config mode

  const uint8_t mn[2] = { (uint8_t)(LD2413_CFG_MIN_MM & 0xFF), (uint8_t)(LD2413_CFG_MIN_MM >> 8) };
  ld2413SendCommand(0x0074, mn, 2);                         // set min distance
  const uint8_t mx[2] = { (uint8_t)(LD2413_CFG_MAX_MM & 0xFF), (uint8_t)(LD2413_CFG_MAX_MM >> 8) };
  ld2413SendCommand(0x0075, mx, 2);                         // set max distance
  const uint8_t rc[2] = { (uint8_t)(LD2413_CFG_REPORT_MS & 0xFF), (uint8_t)(LD2413_CFG_REPORT_MS >> 8) };
  ld2413SendCommand(0x0071, rc, 2);                         // set report cycle

  ld2413SendCommand(0x00FE, nullptr, 0);                    // exit config mode → resume streaming
  delay(200);
}
#endif  // LD2413_CONFIGURE_ON_BOOT

#endif  // ENABLE_SENSOR_LD2413

// ================================================================
//  Sensor auto-detection
//
//  sensorBegin(t) — bring up the hardware for one sensor type so it can be
//  probed or read. sensorProbe(t) — return true if that sensor responds.
//  sensorsDetect() — probe the enabled sensors in order and return the first
//  that answers (or NONE).
// ================================================================
static bool sensorProbe(SensorType t);

static void sensorBegin(SensorType t) {
  switch (t) {
#if defined(ENABLE_SENSOR_LD2413)
    case SENSOR_TYPE_LD2413:
      {
        bool hadSwitch = (PIN_LD2413_PWR >= 0);
        if (hadSwitch) {
          pinMode(PIN_LD2413_PWR, OUTPUT);
          digitalWrite(PIN_LD2413_PWR, HIGH);   // power the radar
        }
        uartEnd();
        UartSensor.begin(LD2413_BAUD, SERIAL_8N1, PIN_LD2413_RX, PIN_LD2413_TX);
        s_uartBegun = true;
        // Skip the boot-settle wait only with POSITIVE evidence the module
        // never lost power: no load-switch fitted (so sensorsShutdown()
        // never cut its supply) AND it was already confirmed running on an
        // earlier wake — s_typeConfirmed only becomes true after a prior
        // successful detection, so this can't fire on a true cold boot or
        // during the initial auto-probe below, where the module's actual
        // power-on timing is unknown and the settle time is still needed.
        bool skipDelay = !hadSwitch && s_typeConfirmed;
        if (!skipDelay) delay(LD2413_BOOT_MS);   // power-up → first data frame
        esp_task_wdt_reset();
      }
#if LD2413_CONFIGURE_ON_BOOT
      ld2413Configure();
      esp_task_wdt_reset();
#endif
      break;
#endif
#if defined(ENABLE_SENSOR_RCWL1670)
    case SENSOR_TYPE_RCWL1670:
      pinMode(PIN_RCWL_TRIG, OUTPUT);
      digitalWrite(PIN_RCWL_TRIG, LOW);
      pinMode(PIN_RCWL_ECHO, INPUT);
      break;
#endif
    default:
      break;
  }
}

static bool sensorProbe(SensorType t) {
  switch (t) {
#if defined(ENABLE_SENSOR_LD2413)
    case SENSOR_TYPE_LD2413: {
      while (UartSensor.available()) UartSensor.read();     // flush stale bytes
      float mm;
      for (uint8_t k = 0; k < 3; k++) {                     // listen for a frame
        esp_task_wdt_reset();
        if (ld2413ReadFrame(mm, 500)) return true;
      }
      return false;
    }
#endif
#if defined(ENABLE_SENSOR_RCWL1670)
    case SENSOR_TYPE_RCWL1670: {
      for (uint8_t k = 0; k < 5; k++) {                     // fire a few pings
        esp_task_wdt_reset();
        digitalWrite(PIN_RCWL_TRIG, LOW);  delayMicroseconds(2);
        digitalWrite(PIN_RCWL_TRIG, HIGH); delayMicroseconds(10);
        digitalWrite(PIN_RCWL_TRIG, LOW);
        unsigned long d = pulseIn(PIN_RCWL_ECHO, HIGH, RCWL1670_ECHO_TIMEOUT_US);
        if (d > 0) {
          float cm = d * 0.0343f / 2.0f;
          if (cm >= RCWL1670_MIN_CM && cm <= RCWL1670_MAX_CM) return true;
        }
        delay(RCWL1670_INTER_SAMPLE_MS);
      }
      return false;
    }
#endif
    default:
      return false;
  }
}

static SensorType sensorsDetect() {
  Serial.println(F("[Sensor] Auto-detecting connected distance sensor..."));
  // Probe order: LD2413 (radar, preferred) → RCWL. They are on independent
  // pins, so the order is simply preference.
  static const SensorType order[] = {
#if defined(ENABLE_SENSOR_LD2413)
    SENSOR_TYPE_LD2413,
#endif
#if defined(ENABLE_SENSOR_RCWL1670)
    SENSOR_TYPE_RCWL1670,
#endif
  };
  for (uint8_t i = 0; i < sizeof(order) / sizeof(order[0]); i++) {
    esp_task_wdt_reset();
    sensorBegin(order[i]);
    if (sensorProbe(order[i])) {
      Serial.printf("[Sensor] Detected: %s\n", sensorTypeName(order[i]));
      return order[i];
    }
    Serial.printf("[Sensor]  ...%s not responding\n", sensorTypeName(order[i]));
  }
  Serial.println(F("[Sensor] No sensor positively detected"));
  return SENSOR_TYPE_NONE;
}

// ================================================================
//  Optional LIS2DW12 mount-tilt monitoring (Wire1, 0x19)
//
//  Detects whether the gauge has moved from its installed orientation — a
//  shifted reference silently biases every reduced depth in a survey. The
//  installed tilt is captured as a baseline on the first read after a cold
//  boot (RTC RAM); a later change beyond TILT_MOVED_DEG raises the flag.
//  UNVERIFIED on hardware — bench-test before trusting it. See config.h.
// ================================================================
#if defined(ENABLE_TILT_LIS2DW12)
RTC_DATA_ATTR static float s_tiltBaselineDeg = -1.0f;

static bool lis2dw12ReadReg(uint8_t reg, uint8_t* buf, uint8_t len) {
  I2CSensors.beginTransmission(LIS2DW12_ADDR);
  I2CSensors.write(reg);
  if (I2CSensors.endTransmission(false) != 0) return false;   // repeated start
  if (I2CSensors.requestFrom((int)LIS2DW12_ADDR, (int)len) != (int)len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)I2CSensors.read();
  return true;
}

static bool lis2dw12WriteReg(uint8_t reg, uint8_t val) {
  I2CSensors.beginTransmission(LIS2DW12_ADDR);
  I2CSensors.write(reg); I2CSensors.write(val);
  return I2CSensors.endTransmission() == 0;
}

// Read the three accelerometer axes (g, ±2g). Returns false if the device is
// absent or isn't a LIS2DW12. Registers: WHO_AM_I 0x0F (=0x44), CTRL6 0x25
// (full scale), CTRL1 0x20 (ODR/mode), OUT_X_L..OUT_Z_H 0x28..0x2D
// (auto-incremented; IF_ADD_INC is on by default).
static bool lis2dw12Read(float& ax, float& ay, float& az) {
  uint8_t who;
  if (!lis2dw12ReadReg(0x0F, &who, 1) || who != 0x44) return false;  // WHO_AM_I
  lis2dw12WriteReg(0x25, 0x00);   // CTRL6: ±2g full scale
  lis2dw12WriteReg(0x20, 0x24);   // CTRL1: 12.5 Hz, high-performance → measuring
  delay(100);                     // let at least one 12.5 Hz sample be produced
  uint8_t d[6];
  bool ok = lis2dw12ReadReg(0x28, d, 6);
  lis2dw12WriteReg(0x20, 0x00);   // back to power-down (µA draw during sleep)
  if (!ok) return false;
  int16_t x = (int16_t)(d[0] | (d[1] << 8));
  int16_t y = (int16_t)(d[2] | (d[3] << 8));
  int16_t z = (int16_t)(d[4] | (d[5] << 8));
  // Data is left-justified in the 16-bit registers; at ±2g full scale that is
  // 16384 LSB per g regardless of 12-/14-bit resolution.
  ax = x / 16384.0f; ay = y / 16384.0f; az = z / 16384.0f;
  return true;
}

// True if the gauge has tilted more than TILT_MOVED_DEG from its baseline.
static bool readMountMoved() {
  float ax, ay, az;
  if (!lis2dw12Read(ax, ay, az)) return false;      // sensor absent → assume ok
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  if (mag < 0.1f) return false;
  float c = az / mag; if (c > 1.0f) c = 1.0f; if (c < -1.0f) c = -1.0f;
  float ang = acosf(c) * 57.2957795f;               // tilt from vertical (deg)
  if (s_tiltBaselineDeg < 0.0f) {                   // first boot → set baseline
    s_tiltBaselineDeg = ang;
    Serial.printf("[Tilt] Baseline set: %.2f deg\n", ang);
    return false;
  }
  bool moved = fabsf(ang - s_tiltBaselineDeg) > TILT_MOVED_DEG;
  Serial.printf("[Tilt] %.2f deg (baseline %.2f)%s\n",
                ang, s_tiltBaselineDeg, moved ? "  *** MOVED ***" : "");
  return moved;
}
#endif  // ENABLE_TILT_LIS2DW12

// ================================================================
//  getBatteryVoltage — averaged, calibrated LiPo voltage (V) at the cell.
// ================================================================
// One averaged, eFuse-calibrated read of the cell (no caching). The ESP32-S3
// SAR ADC is noisy and non-linear, so a single raw reading near BAT_CUTOFF_V
// can falsely trip (or mask) the low-battery path. Average several samples via
// analogReadMilliVolts() (factory-calibrated true mV at the pin) — far better
// than the flat raw/4095 × 3.3 formula.
static float sampleBatteryVoltage() {
  constexpr uint8_t N = 16;
  uint32_t mvSum = 0;
  for (uint8_t i = 0; i < N; i++) mvSum += analogReadMilliVolts(PIN_BAT_ADC);
  float pinMv = (float)mvSum / N;       // averaged voltage at the ADC pin
  return (pinMv / 1000.0f) * 2.0f;      // ×2 for the on-board ÷2 divider
}

float getBatteryVoltage() {
  // Cache the result for the rest of this wake: the node reads the cell twice
  // per wake (an early pre-radio cutoff check and again inside readAllSensors),
  // seconds apart on a resting-then-radio-warm cell — 16 of one resting sample
  // suffice. Reset on every deep-sleep boot (function statics), so each wake
  // still samples afresh. NOTE: this is the RESTING value; a live under-load
  // reading must use getBatteryVoltageFresh() instead (see the flush sag guard).
  static bool  s_cached = false;
  static float s_batV   = 0.0f;
  if (s_cached) return s_batV;
  s_batV   = sampleBatteryVoltage();
  s_cached = true;
  return s_batV;
}

// Uncached read for when a FRESH value matters more than per-wake consistency —
// specifically the pending-flush loaded-voltage guard, which must see the cell
// sagging UNDER WiFi TX. Going through getBatteryVoltage() there would return
// the cached pre-radio resting value and defeat the guard entirely.
float getBatteryVoltageFresh() {
  return sampleBatteryVoltage();
}

// ================================================================
//  readSHT3x — read air temperature + humidity from the SHT3x. Re-begins the
//  sensor once if it has dropped off the bus. Returns false (leaving the
//  outputs untouched) if it's absent or the values fail the sanity thresholds.
// ================================================================
static bool readSHT3x(float& tempC, float& humidity) {
  if (!sht.isConnected()) {
    sht.begin();
    delay(15);
    if (!sht.isConnected()) return false;
  }
  if (!sht.read(false)) return false;
  // Stage into locals so a sanity-check failure really does leave the caller's
  // outputs untouched, as the contract above promises.
  float t = sht.getTemperature();
  float h = sht.getHumidity();
  if (t < TEMP_MIN_C || t > TEMP_MAX_C ||
      h < HUM_MIN_PCT || h > HUM_MAX_PCT) return false;
  tempC    = t;
  humidity = h;
  return true;
}

// ================================================================
//  readRTC — read the DS3231 and format an ISO-8601 string into buf24
//  ("YYYY-MM-DDTHH:MM:SS") plus the Unix epoch. Returns false if the reading
//  is invalid, in which case Lilygo.ino substitutes its estimated epoch.
// ================================================================
static bool readRTC(char* buf24, uint32_t& epochOut) {
  DateTime now = rtc.now();
  if (!now.isValid()) return false;
  snprintf(buf24, 24, "%04d-%02d-%02dT%02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());
  epochOut = now.unixtime();
  return true;
}

// ================================================================
//  readAllSensors
//
//  Pipeline:
//    1. Distance burst → trim each end → mean, dispatched at RUNTIME to the
//       auto-detected sensor (ultrasonic pulses, or LD2413 radar frames)
//    2. SHT3x reads temperature + humidity
//    3. Speed-of-sound correct: corrected = raw × c(T,RH)/0.0343
//       Falls back to last good T/RH if the env sensor misses this wake.
//       SKIPPED for the LD2413 radar (its range is temp/humidity-independent).
//    4. corrected → 1-D Kalman (sleep-scaled Q, spike-gated) → distanceCm
//    5. waterLevelCm = SENSOR_HEIGHT_CM − distanceCm
// ================================================================
SensorReadResult readAllSensors(SensorData& out) {
  SensorReadResult result;

  // Step 1 — raw distance burst, dispatched by the auto-detected sensor.
  float rawDist = -1.0f;
  switch (s_activeType) {
#if defined(ENABLE_SENSOR_RCWL1670)
    case SENSOR_TYPE_RCWL1670: rawDist = readRCWL1670Raw(); break;
#endif
#if defined(ENABLE_SENSOR_LD2413)
    case SENSOR_TYPE_LD2413:   rawDist = readLD2413Raw();   break;
#endif
    default:                   g_burstSd = -1.0f; g_burstN = 0; break;  // NONE
  }
  out.sensorType = s_activeType;
  esp_task_wdt_reset();

  // Step 2 — environment sensor (still read for the payload/log, and — for the
  // ultrasonics — for the speed-of-sound correction below)
  out.envValid = readSHT3x(out.tempC, out.humidity);
  result.envOk = out.envValid;
  if (out.envValid) { s_lastTempC = out.tempC; s_lastHumidity = out.humidity; }

  // Step 3 — speed-of-sound correction (temperature + humidity). Applied to
  // the ULTRASONIC sensors, whose raw distance used a fixed 0.0343 cm/µs
  // constant. The LD2413 RADAR reports a true range (speed of light), so it
  // is skipped for it — decided at runtime from the active sensor type.
  if (s_activeType != SENSOR_TYPE_LD2413 && rawDist > 0.0f) {
    float factor = tempCompFactor(s_lastTempC, s_lastHumidity);
    rawDist *= factor;
    Serial.printf("[Sensor] SoS factor %.4f (T=%.1f C, RH=%.0f%%)\n",
                  factor, s_lastTempC, s_lastHumidity);
  }

  // Step 4 — Kalman filter (Q scaled by sleep duration set via sensorsSetSleepDuration)
  out.distanceRaw   = rawDist;
  out.distanceValid = (rawDist > 0.0f);
  result.distanceOk = out.distanceValid;

  // Adaptive re-detection: a valid read confirms the sensor (promoting a
  // fallback); SENSOR_REDETECT_FAILS consecutive invalid reads from a confirmed
  // sensor force a fresh auto-detect next wake (sensor swapped / failed).
  if (out.distanceValid) {
    s_failStreak = 0;
    if (!s_typeConfirmed) {
      s_typeConfirmed = true;
      Serial.printf("[Sensor] %s confirmed by a valid read\n",
                    sensorTypeName(s_activeType));
    }
  } else if (s_typeConfirmed) {
    if (++s_failStreak >= SENSOR_REDETECT_FAILS) {
      Serial.println(F("[Sensor] Repeated read failures — re-detecting next wake"));
      s_typeConfirmed = false;
      s_failStreak    = 0;
    }
  }

  // Burst QC + mode tag (filled by collapseBurst() during the read above) —
  // travel with the reading so tide-reduction can judge each value's quality.
  out.burstSd    = g_burstSd;
  out.burstN     = g_burstN;
  out.surveyMode = s_surveyMode;
#if defined(ENABLE_TILT_LIS2DW12)
  out.moved      = readMountMoved();   // mount-integrity check (survey QA)
#endif

  if (out.distanceValid) {
    out.distanceCm = kalmanUpdate(rawDist, g_burstSd);
  } else {
    out.distanceCm = -1.0f;
  }

  // Step 5 — water level (uses Kalman-filtered, temp-corrected distance).
  // The speed-of-sound correction can push the distance slightly past
  // SENSOR_HEIGHT_CM (the raw range gate ran before the correction), which
  // would make the level negative — and the Apps Script nulls negative levels,
  // silently hiding the reading. Clamp to 0: "water at/below the zero datum".
  if (out.distanceValid) {
    float wl = SENSOR_HEIGHT_CM - out.distanceCm;
    out.waterLevelCm = (wl < 0.0f) ? 0.0f : wl;
  } else {
    out.waterLevelCm = -1.0f;
  }

  // --- DS3231 ---
  out.rtcValid = readRTC(out.isoTimestamp, out.epochSeconds);
  result.rtcOk = out.rtcValid;
  if (!out.rtcValid) {
    // Placeholder — Lilygo.ino overwrites with RTC_DATA_ATTR epoch estimate
    snprintf(out.isoTimestamp, sizeof(out.isoTimestamp), "UNKNOWN");
  }

  // --- Battery ---
  out.batV     = getBatteryVoltage();
  out.batValid = (out.batV > BAT_PLAUSIBLE_MIN_V && out.batV < BAT_PLAUSIBLE_MAX_V);

  out.capturedAtMs = millis();
  return result;
}
