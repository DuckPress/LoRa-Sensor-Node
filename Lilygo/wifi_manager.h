#pragma once
#include <Arduino.h>
#include <WiFi.h>

// ================================================================
//  wifi_manager — multi-SSID station-mode WiFi with a deep-sleep-aware
//  fast-reconnect cache. Everything that needs the internet (cloud
//  uploads, pending-queue flush, OTA) is built on top of wifiConnect().
// ================================================================

// Bring up WiFi: try the cached AP first (scan-free, sub-second — channel +
// BSSID are remembered in RTC RAM across deep sleep), then fall back to scanning
// each SSID from the secret store in order. Returns true once associated.
bool        wifiConnect();

// Current link state — thin wrapper over WiFi.status(); WL_CONNECTED when up.
wl_status_t wifiStatus();

// Index of the secret-store WiFi network we connected to this wake, or -1.
int8_t      wifiCurrentIndex();

// Signal strength (dBm) of the current link, or 0 when not connected.
int32_t     wifiRSSI();
