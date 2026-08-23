#pragma once
#include "sensors.h"
#include <stdint.h>

// Append the current reading to the failed-upload queue on SD.
// Call on every non-WiFi wake so missed uploads are retried later.
void pendingAppend(const SensorData& data);

// Upload queued entries to GAS (best-effort), oldest-first, then rewrite the
// survivors. Returns the number of entries successfully uploaded.
// Call at the start of every WiFi wake, before uploading the current reading.
//
// batteryV is the pre-radio RESTING battery voltage (Lilygo.ino's earlyBatV);
// the number of entries flushed this wake is scaled to it (see
// flushBudgetForVoltage / config.h's BAT_FLUSH_* tiers) so draining a large
// backlog can't brown out a depleted cell. A live loaded-voltage re-check
// during the flush defers the rest if the cell sags under WiFi load.
// capOverride (0 = none) is a remote hard cap from the Config sheet's
// flush_cap knob: the wake flushes at most min(voltage budget, capOverride).
uint32_t pendingFlush(float batteryV, uint32_t capOverride = 0);

// Per-wake flush budget (entries) for a given resting battery voltage — the
// voltage tiers in config.h. 0 means "too low, skip flushing this wake".
uint32_t flushBudgetForVoltage(float batteryV);

// Return the number of entries currently in the queue (0 if file absent).
uint32_t pendingCount();
