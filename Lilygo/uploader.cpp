#include "uploader.h"
#include "config.h"
#include "gas_upload.h"

// Build the GAS query string from a live SensorData (sentinel values for any
// invalid field) and upload it via gasUpload(). Used for the CURRENT reading on
// a WiFi wake; buffered readings have their own builder in pending.cpp.
bool uploadData(const SensorData& data, int32_t wifiRssi) {
  // Build the query string (everything after '?'); gasUpload() handles the URL,
  // TLS, redirect and timeout. Fields:
  //   dist    = Kalman-filtered distance (cm)
  //   distRaw = trimmed-mean distance before Kalman (cm)
  //   rssi    = WiFi RSSI at connect time (passed in by the caller)
  //   rv/ev   = RTC/env valid flags — same convention as the LoRa payload
  //             (lora_comms.cpp); omitting these left every direct-upload
  //             row without rtc_valid/env_valid, which fails the rv=1 gate
  //             every downstream QC/tide-reduction step relies on.
  char q[512];
  int n = snprintf(q, sizeof(q),
    "ts=%s"
    "&dist=%.2f"
    "&distRaw=%.2f"
    "&wl=%.2f"
    "&temp=%.2f"
    "&hum=%.1f"
    "&bat=%.2f"
    "&rssi=%d"
    "&rv=%d"
    "&ev=%d"
    "&id=%u"
    "&sd=%.2f"
    "&n=%u"
    "&sm=%d"
    "&mv=%d",
    data.isoTimestamp,
    data.distanceValid ? data.distanceCm   : -1.0f,
    data.distanceValid ? data.distanceRaw  : -1.0f,
    data.distanceValid ? data.waterLevelCm : -1.0f,
    data.envValid      ? data.tempC        : -999.0f,
    data.envValid      ? data.humidity     : -1.0f,
    // -1 = invalid, matching the LoRa payload's sentinel; the Apps Script
    // nulls negative batteries but would show a literal 0.00 V as real data.
    data.batValid      ? data.batV         : -1.0f,
    (int)wifiRssi,
    data.rtcValid      ? 1 : 0,
    data.envValid      ? 1 : 0,
    (unsigned)NODE_ID,
    data.burstSd,
    (unsigned)data.burstN,
    data.surveyMode    ? 1 : 0,
    data.moved         ? 1 : 0);

  if (n <= 0 || (size_t)n >= sizeof(q)) {
    Serial.println(F("[Upload] query build failed"));
    return false;
  }

  Serial.printf("[Upload] GET ?%s\n", q);
  return gasUpload(q);
}
