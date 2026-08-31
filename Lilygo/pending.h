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

// Drain the pending queue over LoRa (oldest-first) through the gateway, instead
// of WiFi — so a node with no WiFi can still recover its backlog. Each entry is
// sent as an ordinary LoRa packet (loraSend); a delivered entry is dropped, a
// NO_ACK stops the flush (link down) and keeps the rest. Drains as many as it
// can within LORA_FLUSH_BUDGET_MS. batteryV is the pre-radio resting voltage
// (skipped below BAT_FLUSH_LOW_V). Call only after the real-time reading was
// ACKed. Returns the number sent over LoRa this wake.
uint32_t loraFlushPending(float batteryV);
