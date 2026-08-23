#pragma once
#include "sensors.h"

/**
 * Upload SensorData to the Google Apps Script endpoint via HTTPS GET.
 * GET + query params avoids the GAS 302-redirect POST-body-drop issue.
 * wifiRssi should be captured right after wifiConnect() succeeds (e.g. the
 * caller's cached s_lastWifiRssi) rather than read fresh here — by the time
 * this runs (after NTP sync + a config GET), WiFi modem sleep can make a
 * fresh WiFi.RSSI() call read back 0.
 * Returns true on HTTP 200.
 */
bool uploadData(const SensorData& data, int32_t wifiRssi);
