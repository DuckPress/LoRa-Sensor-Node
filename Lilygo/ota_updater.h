#pragma once
#include <Arduino.h>

/**
 * ota_updater.h — OTA firmware update via GitHub Releases
 *
 * 1. Fetch version.txt from GitHub Releases.
 * 2. Compare against FIRMWARE_VERSION in config.h.
 * 3. If remote is strictly newer, stream the .bin via HTTPUpdate.
 * 4. On a verified flash, record the version ("flashed" in NVS) and reboot.
 * 5. All failures are non-fatal — returns false, firmware continues, and the
 *    same version is retried on the next check (nothing is recorded).
 *
 * The caller (Lilygo.ino) decides WHEN to check: the periodic schedule, or a
 * rate-limited Config request, never on wake #1 and only with enough battery.
 *
 * Partition requirement:
 *   Tools → Partition Scheme → Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
 */

void otaInit();
bool checkForOTAUpdate();
