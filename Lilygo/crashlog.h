#pragma once
#include <Arduino.h>
#include <esp_system.h>

// ================================================================
//  crashlog — make a field crash (PANIC / watchdog reset) diagnosable
//  WITHOUT anyone on the serial port. Two independent mechanisms:
//
//  1. Stage breadcrumb. setup() calls crashStageSet() as it enters each phase
//     of a wake. The byte lives in RTC_NOINIT memory, which — unlike
//     RTC_DATA_ATTR — is NOT re-initialised by a panic/software reset, so
//     after a crash the next boot still knows which phase was running
//     ("it always dies during the LoRa flush"). Guarded by a magic word so a
//     true power-on (garbage RTC RAM) reads as "unknown", not a false stage.
//
//  2. Core dump summary. ESP-IDF writes a core dump to the `coredump` flash
//     partition on every panic (the task watchdog is configured to panic too),
//     which survives the reboot. On the next boot the firmware pulls the
//     SUMMARY — reset reason, exception cause, crashing task, PC and the
//     backtrace addresses — and (a) appends it to /crashlog.csv on the SD card
//     at once, (b) uploads it on the next WiFi wake, then erases the dump.
//     The backtrace is raw addresses; tools/decode_crash.py turns them into
//     file:line against the ELF of that exact build (keep the ELF per release).
// ================================================================

enum CrashStage : uint8_t {
  STAGE_NONE = 0,        // unknown (power-on / never set)
  STAGE_BOOT,
  STAGE_SENSORS_INIT,
  STAGE_SD_INIT,
  STAGE_SECRETS,
  STAGE_LORA_INIT,
  STAGE_READ_SENSORS,
  STAGE_SD_LOG,
  STAGE_LORA_TX,
  STAGE_WIFI_CONNECT,
  STAGE_NTP,
  STAGE_NODECFG,
  STAGE_OTA,
  STAGE_WIFI_FLUSH,
  STAGE_UPLOAD,
  STAGE_LORA_FLUSH,
  STAGE_DISPLAY,
  STAGE_SLEEP,
  STAGE__COUNT
};

const char* crashStageName(uint8_t stage);

// Record the phase the wake is entering. Cheap (one RTC RAM write).
void crashStageSet(uint8_t stage);

// Call at the very top of setup(). Captures the breadcrumb the PREVIOUS run
// left (the stage it was in when it died) and re-arms the breadcrumb for this
// run. Returns that previous stage (STAGE_NONE if unknown, STAGE_SLEEP after a
// normal deep-sleep wake).
uint8_t crashBootBegin(esp_reset_reason_t reason);

// True when a core dump is stored in flash (a panic happened since the last
// erase — possibly several boots ago if no WiFi wake has run since).
bool crashDumpPresent();

// Once per stored dump: append its summary to /crashlog.csv (NVS-guarded so a
// dump that waits several LoRa-only wakes for its WiFi wake is logged once).
void crashLogToSdOnce(const char* isoTs, uint32_t wakeCount);

// On a connected WiFi wake: upload the summary (action=crash). On success the
// dump is erased so the same crash is never reported twice. Returns true if
// uploaded (or nothing to upload).
bool crashUploadIfAny(const char* isoTs);
