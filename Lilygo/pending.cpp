#include "pending.h"
#include "config.h"
#include "gas_upload.h"
#include <SD.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

// ================================================================
//  Convert one CSV line from the pending queue to its batch-upload query.
//
//  Line format (no header, comma-separated):
//    ts,dist_k,dist_raw,wl,temp,hum,bat[,sd,n,sm,mv]
//  The trailing QC fields (burst std-dev, burst count, survey-mode flag,
//  mount-moved flag) are optional so lines queued by older firmware still
//  parse — they default to -1/0/0/0.
//
//  rssi is set to 0 because WiFi was not connected when the reading
//  was taken — the node only knew the LoRa channel quality at that
//  time, which is not stored in the pending queue.
// ================================================================
// Returns false for a corrupt/unparseable line; re-queueing one forever would
// stall the flush, while tidelog.csv still holds the complete record.
static bool pendingLineToQuery(const String& line, String& queryOut) {
  if (line.length() < 5) return false;

  char  ts[24] = { 0 };
  float dist   = -1.0f, distRaw = -1.0f, wl  = -1.0f;
  float temp   = -999.0f, hum   = -1.0f, bat = -1.0f;
  float sd     = -1.0f;
  int   bn     = 0, sm = 0, mv = 0;

  int parsed = sscanf(line.c_str(), "%23[^,],%f,%f,%f,%f,%f,%f,%f,%d,%d,%d",
                      ts, &dist, &distRaw, &wl, &temp, &hum, &bat,
                      &sd, &bn, &sm, &mv);
  if (parsed < 7 || ts[0] == '\0') return false;

  char q[512];
  int n = snprintf(q, sizeof(q),
    "ts=%s"
    "&dist=%.2f"
    "&distRaw=%.2f"
    "&wl=%.2f"
    "&temp=%.2f"
    "&hum=%.1f"
    "&bat=%.2f"
    "&rssi=0"
    "&id=%u"
    "&sd=%.2f"
    "&n=%d"
    "&sm=%d"
    "&mv=%d",
    ts, dist, distRaw, wl, temp, hum, bat,
    (unsigned)NODE_ID, sd, bn, sm, mv);
  if (n <= 0 || (size_t)n >= sizeof(q)) return false;

  queryOut = q;
  return true;
}

// ================================================================
//  pendingAppend — append the current reading to /pending.csv (the WiFi-backup
//  queue), called when LoRa didn't confirm delivery. Enforces the cap, dropping
//  the newest reading when full (tidelog.csv on SD keeps the complete record).
// ================================================================
void pendingAppend(const SensorData& data) {
  // Enforce the cap before opening the file for append so we never
  // exceed PENDING_MAX_ENTRIES even when the gateway is offline for
  // an extended period.
  if (pendingCount() >= PENDING_MAX_ENTRIES) {
    // Queue full: drop this (newest) reading rather than the backlog.
    // No data is truly lost — tidelog.csv on SD holds the complete record.
    Serial.printf("[Pending] Queue full (%u) — dropping this reading "
                  "until a WiFi flush succeeds\n", PENDING_MAX_ENTRIES);
    return;
  }

  // Heal a torn tail from an interrupted previous append: if the file doesn't
  // end in '\n', the new record would merge into the partial line and corrupt
  // both.
  bool needsNewline = false;
  if (SD.exists(PENDING_FILENAME)) {
    File chk = SD.open(PENDING_FILENAME, FILE_READ);
    if (chk) {
      size_t sz = chk.size();
      if (sz > 0 && chk.seek(sz - 1) && chk.read() != '\n') needsNewline = true;
      chk.close();
    }
  }

  File f = SD.open(PENDING_FILENAME, FILE_APPEND);
  if (!f) {
    Serial.println(F("[Pending] Open for append failed"));
    return;
  }

  if (needsNewline) f.print('\n');
  // Leading "0|" is the retry counter pendingFlush() maintains (see
  // PENDING_MAX_RETRIES) — dropped entries are counted this way instead of
  // being kept forever.
  f.print("0|");
  f.printf("%s,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%u,%d,%d\n",
    data.isoTimestamp,
    data.distanceValid ? data.distanceCm   : -1.0f,
    data.distanceValid ? data.distanceRaw  : -1.0f,
    data.distanceValid ? data.waterLevelCm : -1.0f,
    data.envValid      ? data.tempC        : -999.0f,
    data.envValid      ? data.humidity     : -1.0f,
    // -1 = invalid, matching the LoRa payload sentinel (GAS nulls negatives).
    data.batValid      ? data.batV         : -1.0f,
    data.burstSd,
    (unsigned)data.burstN,
    data.surveyMode    ? 1 : 0,
    data.moved         ? 1 : 0);
  f.close();
}

// ================================================================
//  quarantinePendingFile — move a corrupt queue aside instead of choking on it.
//  A torn write or FAT damage can leave a record with no newline; rather than
//  let readStringUntil() pull an unbounded blob into RAM or jam the flush
//  forever, rename the file to PENDING_BAD_FILENAME (kept for forensics) and
//  start fresh. tidelog.csv remains the complete record, so nothing is lost.
// ================================================================
static void quarantinePendingFile() {
  SD.remove(PENDING_BAD_FILENAME);              // keep only the latest bad copy
  if (SD.rename(PENDING_FILENAME, PENDING_BAD_FILENAME)) {
    Serial.println(F("[Pending] Corrupt queue quarantined to /pending.bad — "
                     "starting fresh (tidelog.csv keeps the full record)"));
  } else {
    SD.remove(PENDING_FILENAME);                // rename failed — drop it outright
    Serial.println(F("[Pending] Corrupt queue removed (rename failed)"));
  }
}

// ================================================================
//  flushBudgetForVoltage — map a resting battery voltage to how many entries
//  may be flushed this wake (see config.h's BAT_FLUSH_* tiers). Scaling the
//  drain to the cell's state of charge keeps a big backlog from browning out a
//  depleted battery; returning 0 near cutoff lets the cell recover instead.
// ================================================================
uint32_t flushBudgetForVoltage(float batteryV) {
  // An implausible reading means USB-only / no real cell (a floating ADC) —
  // there's no battery to protect, so allow the full budget.
  if (batteryV <= BAT_PLAUSIBLE_MIN_V || batteryV > BAT_PLAUSIBLE_MAX_V)
    return PENDING_FLUSH_MAX_PER_WAKE;
  if (batteryV >= BAT_FLUSH_FULL_V) return PENDING_FLUSH_MAX_PER_WAKE;
  if (batteryV >= BAT_FLUSH_MED_V)  return PENDING_FLUSH_MEDIUM;
  if (batteryV >= BAT_FLUSH_LOW_V)  return PENDING_FLUSH_SMALL;
  return 0;   // below BAT_FLUSH_LOW_V — defer the flush, let the cell recover
}

// ================================================================
//  pendingFlush — on a WiFi wake, upload queued readings oldest-first (up to
//  a voltage-scaled per-wake budget), rewriting failed/over-budget entries
//  back to the queue for the next wake. Returns the number uploaded.
// ================================================================
uint32_t pendingFlush(float batteryV, uint32_t capOverride) {
  static const char* PENDING_TMP = "/pending.tmp";

  // Recover from a crash between the remove and the rename at the bottom of a
  // previous flush: the queue file is gone but the fully-written temp file
  // still holds the kept entries. Do this BEFORE the budget check so an
  // interrupted flush is always healed even on a wake that then defers.
  if (!SD.exists(PENDING_FILENAME) && SD.exists(PENDING_TMP)) {
    SD.rename(PENDING_TMP, PENDING_FILENAME);
    Serial.println(F("[Pending] Recovered queue from interrupted flush"));
  }

  if (!SD.exists(PENDING_FILENAME)) return 0;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[Pending] No WiFi — flush skipped"));
    return 0;
  }

  // Voltage-scaled per-wake budget, then a remote hard cap on top (flush_cap
  // from the Config sheet, 0 = none). 0 = cell too low to risk the flush
  // burst; skip WITHOUT rewriting the file (churning the whole queue to a temp
  // file and back is itself avoidable SD work on a struggling node).
  uint32_t budget = flushBudgetForVoltage(batteryV);
  if (capOverride > 0 && capOverride < budget) budget = capOverride;
  if (budget == 0) {
    Serial.printf("[Pending] Battery %.2fV low — flush deferred to conserve charge\n",
                  batteryV);
    return 0;
  }

  File in = SD.open(PENDING_FILENAME, FILE_READ);
  if (!in) return 0;

  // Entries beyond this wake's budget are streamed to a temp file (oldest
  // first stays at the head) and re-queued, bounding awake time per flush.
  SD.remove(PENDING_TMP);                       // stale partial temp, if any
  File out = SD.open(PENDING_TMP, FILE_WRITE);
  if (!out) {
    // Without the temp file, failed/deferred entries couldn't be preserved.
    // Abort the whole flush and retry next wake instead of risking the
    // backlog; nothing has been uploaded or deleted yet.
    Serial.println(F("[Pending] Temp open failed — flush deferred"));
    in.close();
    return 0;
  }

  uint32_t uploaded  = 0;
  uint32_t attempted = 0;
  uint32_t kept      = 0;   // failed + over-budget entries preserved for retry
  uint32_t dropped   = 0;   // corrupt lines discarded
  bool stopAttempts  = false;
  bool sagChecked    = false;  // loaded-voltage re-check done once, after batch 1

  String batchRows;
  batchRows.reserve(PENDING_BATCH_MAX_BODY);
  String batchData[PENDING_BATCH_ENTRIES];
  uint8_t batchRetries[PENDING_BATCH_ENTRIES] = { 0 };
  uint8_t batchCount = 0;

  auto flushBatch = [&]() {
    if (batchCount == 0) return;
    esp_task_wdt_reset();
    uint16_t badRows = 0;
    const bool ok = gasUploadBatch(batchRows, batchCount, badRows);
    attempted += batchCount;
    if (ok) {
      uploaded += batchCount - badRows;
      dropped += badRows;
      if (badRows > 0) {
        Serial.printf("[Pending] GAS rejected %u invalid batch row(s)\n", badRows);
      }
    } else {
      stopAttempts = true;
      for (uint8_t i = 0; i < batchCount; i++) {
        if (batchRetries[i] + 1 >= PENDING_MAX_RETRIES) {
          Serial.printf("[Pending] Dropping entry after %u failed attempts: %s\n",
                        batchRetries[i] + 1, batchData[i].c_str());
          dropped++;
        } else {
          out.print(batchRetries[i] + 1); out.print('|');
          out.print(batchData[i]); out.print('\n'); kept++;
        }
      }
    }
    batchRows = "";
    batchCount = 0;
    esp_task_wdt_reset();

    // Loaded-voltage guard (once, right after the first real upload burst).
    // The budget above is set from the pre-radio RESTING voltage; this reads
    // the cell while WiFi is still warm from the TLS batch we just sent, so a
    // resting-fine-but-weak cell that sags under load is caught here and the
    // rest of the flush is deferred (see BAT_FLUSH_SAG_FLOOR_V). Only meaningful
    // for a real cell — an implausible USB-only reading is ignored.
    if (!sagChecked) {
      sagChecked = true;
      // Uncached read — getBatteryVoltage() would hand back the cached pre-radio
      // RESTING value and never see the sag this guard exists to catch.
      float loadedV = getBatteryVoltageFresh();
      if (loadedV > BAT_PLAUSIBLE_MIN_V && loadedV < BAT_FLUSH_SAG_FLOOR_V) {
        Serial.printf("[Pending] Battery sagged to %.2fV under load — deferring rest\n",
                      loadedV);
        stopAttempts = true;
      }
    }

    delay(300);
  };

  while (in.available()) {
    // Every line does at least an SD read here, even lines that only get
    // deferred (over budget / stopAttempts) below — flushBatch() feeds the
    // watchdog while actively uploading, but a large backlog's deferred tail
    // was being copied line-by-line with no feed at all, and could run long
    // enough on its own to trip the 60 s watchdog (see the pendingCount()
    // fix above for the same underlying issue).
    esp_task_wdt_reset();
    String line = in.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    // Corruption guard: a valid record is well under PENDING_MAX_LINE_LEN. An
    // oversized "line" means a torn write / FAT damage left a record with no
    // newline — quarantine the whole queue rather than trust the rest of it.
    if (line.length() >= PENDING_MAX_LINE_LEN) {
      Serial.printf("[Pending] Oversized line (%u B) — queue corrupt\n",
                    (unsigned)line.length());
      in.close();
      out.close();
      SD.remove(PENDING_TMP);
      quarantinePendingFile();
      return uploaded;
    }

    // Parse the "N|data" retry-count prefix (see PENDING_MAX_RETRIES).
    // Tolerate lines written before this counter existed (no recognisable
    // prefix) by treating them as retry 0.
    uint8_t retries = 0;
    String  data    = line;
    int     bar     = line.indexOf('|');
    if (bar > 0 && bar <= 3) {
      String prefix = line.substring(0, bar);
      bool   numeric = true;
      for (unsigned int k = 0; k < prefix.length(); k++) {
        if (!isDigit(prefix[k])) { numeric = false; break; }
      }
      if (numeric) { retries = (uint8_t)prefix.toInt(); data = line.substring(bar + 1); }
    }

    if (stopAttempts || attempted >= budget) {
      // Over this wake's (voltage-scaled) budget, or a sag/failure stopped the
      // flush — defer to the next WiFi wake.
      out.print(retries); out.print('|'); out.print(data); out.print('\n'); kept++;
      continue;
    }

    // Each entry is an HTTPS GET (seconds) + settle delay; reset the
    // watchdog every entry to avoid a panic reset mid-recovery.
    String query;
    if (!pendingLineToQuery(data, query)) {
      // Corrupt line (torn write): dropping it is the only way the flush can
      // ever get past it. No HTTP happened, so it doesn't burn the budget.
      Serial.printf("[Pending] Dropping corrupt entry: %s\n", data.c_str());
      dropped++;
      continue;
    }
    if (batchCount > 0 && batchRows.length() + 1 + query.length() > PENDING_BATCH_MAX_BODY) {
      flushBatch();
      if (stopAttempts) {
        out.print(retries); out.print('|'); out.print(data); out.print('\n'); kept++;
        continue;
      }
    }
    if (batchCount > 0) batchRows += '\n';
    batchRows += query;
    batchData[batchCount] = data;
    batchRetries[batchCount] = retries;
    batchCount++;

    const uint32_t remainingBudget = budget - attempted;
    if (batchCount >= PENDING_BATCH_ENTRIES || batchCount >= remainingBudget) {
      flushBatch();
    }
  }
  flushBatch();
  in.close();
  out.close();

  // Replace the queue with the kept remainder (failed + deferred), or clear
  // it. A crash between the remove and the rename is healed by the recovery
  // check at the top of the next flush.
  SD.remove(PENDING_FILENAME);
  if (kept > 0 && SD.exists(PENDING_TMP)) {
    SD.rename(PENDING_TMP, PENDING_FILENAME);
    Serial.printf("[Pending] Flushed %lu/%lu this wake; %lu still queued, %lu corrupt dropped\n",
                  (unsigned long)uploaded, (unsigned long)attempted,
                  (unsigned long)kept, (unsigned long)dropped);
  } else {
    SD.remove(PENDING_TMP);
    Serial.printf("[Pending] Flushed %lu/%lu  queue cleared (%lu corrupt dropped)\n",
                  (unsigned long)uploaded, (unsigned long)attempted,
                  (unsigned long)dropped);
  }
  return uploaded;
}

// ================================================================
//  pendingCount — number of queued entries (newline count), 0 if no file.
//
//  Reads in chunks rather than byte-by-byte: at PENDING_MAX_ENTRIES (7,500)
//  the file can be ~600 KB, and a single-byte f.read() per SPI transaction
//  over that many bytes can run long enough to trip the task watchdog with
//  no feed in between — which panics (trigger_panic=true) and reboots the
//  node mid-scan.
// ================================================================
uint32_t pendingCount() {
  if (!SD.exists(PENDING_FILENAME)) return 0;

  File f = SD.open(PENDING_FILENAME, FILE_READ);
  if (!f) return 0;

  uint32_t count = 0;
  uint8_t  buf[256];
  int      n;
  while ((n = f.read(buf, sizeof(buf))) > 0) {
    for (int i = 0; i < n; i++) {
      if (buf[i] == '\n') count++;
    }
    esp_task_wdt_reset();
  }
  f.close();
  return count;
}
