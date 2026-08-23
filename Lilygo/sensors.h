#pragma once
#include <Arduino.h>

// ----------------------------------------------------------------
//  Distance-sensor type — which physical sensor a reading came from.
//  All enabled drivers are compiled in; the active one is auto-detected
//  at runtime (see sensorsDetect() / sensorsActiveType()).
// ----------------------------------------------------------------
enum SensorType : uint8_t {
  SENSOR_TYPE_NONE = 0,
  SENSOR_TYPE_RCWL1670,
  SENSOR_TYPE_LD2413,
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

  // Which physical sensor produced this reading (auto-detected).
  SensorType sensorType = SENSOR_TYPE_NONE;

  // Mount integrity: true if the optional tilt sensor detects the gauge has
  // moved from its installed baseline (a shifted reference silently biases
  // every reduced depth in a survey). Always false when tilt is not enabled.
  bool     moved = false;

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

// The distance sensor auto-detected as connected this session (cached across
// deep sleep). SENSOR_TYPE_NONE until the first detection completes.
SensorType       sensorsActiveType();
const char*      sensorTypeName(SensorType t);

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
