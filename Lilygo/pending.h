#pragma once
#include "sensors.h"
#include <stdint.h>

// Append a reading to the backlog queue on SD (/pending.csv). Called for any
// reading the gateway didn't ACK and that wasn't uploaded directly, so it is
// retried later by one of the flushes below.
void pendingAppend(const SensorData& data);

// Upload queued entries to GAS (best-effort), oldest-first, then rewrite the
// survivors. Returns the number of entries successfully uploaded.
// Call on a connected WiFi wake, before uploading the current reading.
//
// batteryV is the pre-radio RESTING battery voltage (Lilygo.ino's earlyBatV);
// the number of entries flushed this wake is scaled to it (see
// flushBudgetForVoltage / config.h's BAT_FLUSH_* tiers) so draining a large
// backlog can't brown out a depleted cell. A live loaded-voltage re-check
// during the flush defers the rest if the cell sags under WiFi load.
// capOverride (0 = none) is a remote hard cap from the Config sheet's
// flush_cap knob: the wake flushes at most min(voltage budget, capOverride).
// A failed upload never drops an entry; only rows the cloud rejects as invalid
// are removed.
uint32_t pendingFlush(float batteryV, uint32_t capOverride = 0);

// Per-wake flush budget (entries) for a given resting battery voltage — the
// voltage tiers in config.h. 0 means "too low, skip flushing this wake".
uint32_t flushBudgetForVoltage(float batteryV);

// Return the number of entries currently in the queue (0 if file absent).
uint32_t pendingCount();

// Drain the pending queue over LoRa (oldest-first) through the gateway, instead
// of WiFi — so a node with no WiFi can still recover its backlog. Each entry is
// sent as an ordinary LoRa packet flagged as a backlog re-send ("bf":1); a
// delivered entry is dropped, a NO_ACK stops the flush and keeps everything.
// Drains as many as it can within LORA_FLUSH_BUDGET_MS, and at most
// capOverride entries when that remote flush_cap is set (0 = none). batteryV is
// the pre-radio resting voltage (skipped below BAT_FLUSH_LOW_V). Call only
// after the real-time reading was ACKed. Returns the number sent over LoRa.
uint32_t loraFlushPending(float batteryV, uint32_t capOverride = 0);
