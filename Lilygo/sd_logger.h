#pragma once
#include "sensors.h"

// Mount the MicroSD card (one retry on failure), create /tidelog.csv with its
// header row if absent, and verify it is appendable. Call once per wake.
void sdInit();

// Append one reading as a CSV row to /tidelog.csv. This is the node's complete,
// authoritative record — written EVERY wake, before the battery cutoff and any
// TX, so even dropped/low-battery readings are captured. Returns false if the
// card isn't ready or the write failed.
bool sdLog(const SensorData& data, uint32_t wakeCount, int32_t wifiRssi = 0);

// True if the card mounted and the log file is writable.
bool sdReady();

// Append one line to /bootlog.csv recording a reset reason (called only for
// real reboots, not deep-sleep wakes), so brownouts/panics are visible later.
bool sdLogBootEvent(const char* reason, uint32_t wakeCount, const char* isoTs);
