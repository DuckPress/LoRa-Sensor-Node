#pragma once
#include <Arduino.h>

// ----------------------------------------------------------------
//  Distance-sensor type — which physical sensor a reading came from.
//  All enabled drivers are compiled in and EVERY one is detected
//  independently at runtime; all present sensors are read each wake and
//  the first valid one in preference order (radar, then ultrasonic) feeds
//  the level pipeline (see sensorsInit() / sensorsActiveType()).
// ----------------------------------------------------------------
enum SensorType : uint8_t {
  SENSOR_TYPE_NONE = 0,
  SENSOR_TYPE_RCWL1670,
  SENSOR_TYPE_LD2413,
  SENSOR_TYPE_COUNT        // number of entries above (array sizing only)
};

// ----------------------------------------------------------------
//  Aggregated reading from all sensors
// ----------------------------------------------------------------
struct SensorData {
  // Ultrasonic — raw = trimmed-mean of 20 samples
  //              distanceCm = Kalman-filtered (persists across deep sleep)
  float    distanceRaw   = -1.0f;  // trimmed-mean distance before Kalman
  float    distanceCm    = -1.0f;  // Kalman-filtered distance (transmitted)
  bool     distanceValid = false;

  // Derived water level: SENSOR_HEIGHT_CM - distanceCm (Kalman)
  float    waterLevelCm  = -1.0f;

  // SHT3x
  float    tempC    = -999.0f;
  float    humidity = -1.0f;
  bool     envValid = false;

  // DS3231 RTC
  char     isoTimestamp[24] = { 0 };  // "YYYY-MM-DDTHH:MM:SS\0"
  bool     rtcValid         = false;
  uint32_t epochSeconds     = 0;

  // Battery voltage (T3-S3 onboard ADC, ÷2 divider → ×2 for true voltage)
  float    batV      = 0.0f;
  bool     batValid  = false;

  // Burst quality (per-reading QC for survey-grade use):
  //   burstSd — standard deviation (cm) of ALL in-range samples in the burst,
  //             computed BEFORE trimming (the spread IS the wave/noise state
  //             that the trimmed mean deliberately hides). -1 = unknown (<2
  //             samples).
  //   burstN  — number of in-range samples that contributed.
  float    burstSd = -1.0f;
  uint8_t  burstN  = 0;

  // Was this reading taken in survey mode (fixed cadence, long burst)?
  // Carried in the payload/log so tide-reduction can identify survey data.
  bool     surveyMode = false;

  // Which physical sensor fed the level pipeline this wake (the PRIMARY):
  // the radar when its burst was valid, else the ultrasonic. NONE when
  // neither produced a reading.
  SensorType sensorType = SENSOR_TYPE_NONE;

  // Dual-sensor build: BOTH distance sensors are read every wake. The primary
  // (sensorType) is what distanceRaw/distanceCm/waterLevelCm came from; the
  // per-sensor trimmed means are kept here so the two can be cross-checked in
  // the log/cloud. -1 = that sensor is absent or its burst was empty.
  float    distanceRadarCm = -1.0f;  // LD2413 trimmed mean (cm), true range
  float    distanceUsCm    = -1.0f;  // RCWL-1670 trimmed mean (cm), speed-of-sound corrected
  float    usBurstSd       = -1.0f;  // ultrasonic burst std-dev (cm), -1 = unknown
  uint8_t  usBurstN        = 0;      // ultrasonic in-range sample count

  // Mount integrity: true if the optional tilt sensor detects the gauge has
  // moved from its installed baseline (a shifted reference silently biases
  // every reduced depth in a survey). Always false when no tilt sensor is fitted.
  bool     moved = false;
  // Tilt from vertical (deg) from the LIS2DW12; -1 when the sensor is absent.
  float    tiltDeg = -1.0f;
  // Station air pressure (hPa) from the BMP280/BME280; -1 when absent.
  float    pressureHpa = -1.0f;

  // Freshness — set to millis() at end of readAllSensors()
  uint32_t capturedAtMs = 0;
};

// ----------------------------------------------------------------
//  Per-subsystem read outcome
// ----------------------------------------------------------------
struct SensorReadResult {
  bool distanceOk = false;
  bool envOk      = false;
  bool rtcOk      = false;

  // Kept for callers that need the historical convenience method: a valid
  // tide reading requires distance, not merely temperature/humidity telemetry.
  bool anyValid() const { return distanceOk; }
  bool allValid() const { return distanceOk && envOk; }
};

// ----------------------------------------------------------------
//  Public API
// ----------------------------------------------------------------

void             sensorsInit();
SensorReadResult readAllSensors(SensorData& out);
float            getBatteryVoltage();       // cached per-wake resting value
float            getBatteryVoltageFresh();  // uncached — for under-load sampling

// The PRIMARY distance sensor for this wake: the first confirmed sensor in
// preference order (LD2413 radar, then RCWL-1670 ultrasonic). Detection state
// is cached across deep sleep; SENSOR_TYPE_NONE until a detection completes.
SensorType       sensorsActiveType();
const char*      sensorTypeName(SensorType t);
// True if that sensor has been positively detected (confirmed) this session.
bool             sensorPresent(SensorType t);
// Short label of everything detected, e.g. "LD2413+RCWL-1670" (for the
// splash/boot log). "NONE" if nothing has answered yet.
const char*      sensorsPresentName();

// Call once per wake, before readAllSensors(), with the duration of the
// previous deep-sleep in seconds. The value accumulates until the Kalman
// filter next incorporates a measurement, so its process noise Q covers the
// full elapsed gap even across invalid or spike-held wakes.
void             sensorsSetSleepDuration(uint32_t secs);

// Survey mode (bathymetric-reference operation): call before readAllSensors().
// When on, the LD2413 read uses the longer SURVEY_* burst (a proper
// wave-averaging window) and the reading is tagged surveyMode=1. The fixed
// wake cadence itself is handled by Lilygo.ino.
void             sensorsSetSurveyMode(bool on);

// Correct the DS3231 from a trusted UTC epoch (e.g. NTP) when it has drifted
// more than maxSkewSec. Returns true if the RTC was (re)set. Only meaningful
// while the sensor I2C bus is up.
bool             rtcSyncIfDrifted(uint32_t localEpoch, int32_t maxSkewSec);  // local = UTC+8

// End Wire1 (sensor I2C bus) before deep sleep to stop current leaking
// through the pull-up resistors.
void             sensorsShutdown();

// ---- Optional tilt (LIS2DW12) / barometer (BMP280/BME280), runtime-detected ----
bool tiltPresent();            // valid after sensorsInit()
bool baroPresent();
void tiltResetBaseline();      // forget the installed baseline (re-levelled)
// Tell the sensor layer this wake was caused by the accelerometer's INT1 line
// (motion wake); readAllSensors() then reports moved=1 for this reading.
void sensorsSetMotionWake(bool motion);
// Leave the LIS2DW12 running in low-power mode with its wake-up interrupt
// armed, so a knock/tilt during deep sleep pulls PIN_TILT_INT high. Call
// right before deep sleep (after the I2C bus is otherwise done). No-op when
// the sensor is absent or PIN_TILT_INT < 0. Returns true if armed.
bool tiltArmMotionWake();
