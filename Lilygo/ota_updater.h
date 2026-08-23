#pragma once
#include <Arduino.h>

/**
 * ota_updater.h — OTA firmware update via GitHub Releases
 *
 * 1. Fetch version.txt from GitHub Releases.
 * 2. Compare against FIRMWARE_VERSION in config.h.
 * 3. If remote is strictly newer, stream the .bin via HTTPUpdate.
 * 4. On success the ESP32 reboots automatically.
 * 5. All failures are non-fatal — returns false, firmware continues.
 *
 * Partition requirement:
 *   Tools → Partition Scheme → Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
 */

void otaInit();
bool checkForOTAUpdate();
