#include "pending.h"
#include "config.h"
#include "gas_upload.h"
#include "lora_comms.h"   // loraSend() — LoRa backlog flush path (loraFlushPending)
#include <SD.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

// Consecutive LoRa-flush wakes whose FIRST send went unacknowledged even though
// the live reading was ACKed moments earlier. A healthy link makes that rare, so
// a streak points at the head entry itself; after LORA_FLUSH_SKIP_AFTER it is
// moved to the tail so the rest of the backlog can drain. RTC RAM: spans wakes.
RTC_DATA_ATTR static uint8_t s_loraHeadFails = 0;

// ================================================================
//  Queue line format (no header, one reading per line):
//
//    N|ts,dist_k,dist_raw,wl,temp,hum,bat,sd,n,sm,mv,tl,pr,st,du,rv
//
//  N is a legacy per-line counter prefix, kept so the format stays readable by
//  older firmware; it no longer decides anything. Every field after `bat` was
//  added over time and is optional when READING, so lines queued by older
//  firmware still parse:
//    sd,n,sm,mv (QC) → -1,0,0,0     tl,pr (>= 1.2.9) → -1,-1
//    st,du (>= 1.2.10) → 0,-1       rv (>= 1.2.11)   → see queuedRtcValid()
// ================================================================
struct QueuedReading {
  char  ts[24];
  bool  tsOk;                                   // ts is a well-formed clock stamp
  float dist, distRaw, wl, temp, hum, bat, sd, tl, pr, du;
  int   bn, sm, mv, st;
  int   rv;                                     // -1 = line predates the rv field
};

// "YYYY-MM-DDTHH:MM:SS" with digits where digits belong — a real clock stamp,
// not "UNKNOWN" or a torn/corrupt fragment.
static bool tsWellFormed(const char* ts) {
  if (strlen(ts) != 19) return false;
  for (int i = 0; i < 19; i++) {
    const char c = ts[i];
    switch (i) {
      case 4: case 7:   if (c != '-') return false; break;
      case 10:          if (c != 'T') return false; break;
      case 13: case 16: if (c != ':') return false; break;
      default:          if (c < '0' || c > '9') return false; break;
    }
  }
  return true;
}

// Parse the data part of one queue line (after the "N|" prefix). Returns false
// for a corrupt/unparseable line. A malformed timestamp is replaced by
// "UNKNOWN" so nothing but a clean stamp (or that sentinel) is ever sent on.
static bool parseQueuedLine(const String& line, QueuedReading& q) {
  if (line.length() < 5) return false;
  memset(&q, 0, sizeof(q));
  q.dist = q.distRaw = q.wl = -1.0f;
  q.temp = -999.0f; q.hum = -1.0f; q.bat = -1.0f;
  q.sd = -1.0f; q.tl = -1.0f; q.pr = -1.0f; q.du = -1.0f;
  q.rv = -1;
  int parsed = sscanf(line.c_str(), "%23[^,],%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%f,%f,%d,%f,%d",
                      q.ts, &q.dist, &q.distRaw, &q.wl, &q.temp, &q.hum, &q.bat,
                      &q.sd, &q.bn, &q.sm, &q.mv, &q.tl, &q.pr, &q.st, &q.du, &q.rv);
  if (parsed < 7 || q.ts[0] == '\0') return false;
  q.tsOk = tsWellFormed(q.ts);
  if (!q.tsOk) strcpy(q.ts, "UNKNOWN");
  return true;
}

// Was the queued reading's clock trusted when it was taken? Lines from firmware
// >= 1.2.11 record it (rv). Older lines don't: a well-formed timestamp is taken
// as valid (what the pre-1.2.11 LoRa flush already assumed); "UNKNOWN" or a
// torn stamp never is.
static int queuedRtcValid(const QueuedReading& q) {
  if (!q.tsOk) return 0;
  return (q.rv >= 0) ? (q.rv ? 1 : 0) : 1;
}

// ================================================================
//  Build the batch-upload query for one queued reading. rssi=0: WiFi wasn't
//  up when it was taken. rv/ev carry the clock/env validity exactly like a
//  direct upload — without rv the cloud treats the row's node timestamp as
//  untrusted, plotting a recovered reading at its UPLOAD time and excluding it
//  from tide reduction. bf=1 marks it as a backlog re-send.
//  Returns false for a corrupt/unparseable line.
// ================================================================
static bool pendingLineToQuery(const String& line, String& queryOut) {
  QueuedReading q;
  if (!parseQueuedLine(line, q)) return false;

  char buf[512];
  int n = snprintf(buf, sizeof(buf),
    "ts=%s"
    "&dist=%.2f"
    "&distRaw=%.2f"
    "&wl=%.2f"
    "&temp=%.2f"
    "&hum=%.1f"
    "&bat=%.2f"
    "&rssi=0"
    "&rv=%d"
    "&ev=%d"
    "&id=%u"
    "&sd=%.2f"
    "&n=%d"
    "&sm=%d"
    "&mv=%d"
    "&tl=%.1f"
    "&pr=%.1f"
    "&st=%d"
    "&du=%.2f"
    "&bf=1",
    q.ts, q.dist, q.distRaw, q.wl, q.temp, q.hum, q.bat,
    queuedRtcValid(q), (q.temp > -100.0f) ? 1 : 0, (unsigned)NODE_ID,
    q.sd, q.bn, q.sm, q.mv, q.tl, q.pr, q.st, q.du);
  if (n <= 0 || (size_t)n >= sizeof(buf)) return false;

  queryOut = buf;
  return true;
}

// ================================================================
//  pendingLineToSensorData — reconstruct a SensorData from one queue line so it
//  can be re-sent over LoRa. Validity flags come from the stored sentinels
//  (-1 / -999) and the stored rv, the same way the live reading set them.
//  backlog=true makes the payload carry "bf":1 (see SensorData::backlog).
// ================================================================
static bool pendingLineToSensorData(const String& line, SensorData& d) {
  QueuedReading q;
  if (!parseQueuedLine(line, q)) return false;

  d = SensorData();   // start from defaults
  strncpy(d.isoTimestamp, q.ts, sizeof(d.isoTimestamp) - 1);
  d.distanceRaw   = q.distRaw;
  d.distanceCm    = q.dist;
  d.waterLevelCm  = q.wl;
  d.distanceValid = (q.dist >= 0.0f);     // -1 sentinel => flagged (C1) reading
  d.tempC         = q.temp;
  d.humidity      = q.hum;
  d.envValid      = (q.temp > -100.0f);   // -999 sentinel => env invalid
  d.batV          = q.bat;
  d.batValid      = (q.bat > 0.0f);
  d.burstSd       = q.sd;
  d.burstN        = (uint8_t)q.bn;
  d.surveyMode    = (q.sm != 0);
  d.moved         = (q.mv != 0);
  d.tiltDeg       = q.tl;
  d.pressureHpa   = q.pr;
  // Dual-sensor fields (node >= 1.2.10). Older lines leave st=0/du=-1; the
  // radar value is recoverable from distRaw when st says the radar was primary.
  d.sensorType      = (q.st > 0 && q.st < SENSOR_TYPE_COUNT) ? (SensorType)q.st : SENSOR_TYPE_NONE;
  d.distanceUsCm    = q.du;
  d.distanceRadarCm = (d.sensorType == SENSOR_TYPE_LD2413) ? q.distRaw : -1.0f;
  d.rtcValid      = (queuedRtcValid(q) == 1);
  d.backlog       = true;
  return true;
}

// ================================================================
//  Read one '\n'-terminated line into `out` (newline stripped), never storing
//  more than maxLen characters — a torn write or FAT damage can leave a
//  "line" with no newline, which an unbounded reader would pull into RAM whole.
//  Returns 1 = got a line, 0 = end of file, -1 = line exceeds maxLen (corrupt).
// ================================================================
static int readLineBounded(File& f, String& out, size_t maxLen) {
  out = "";
  if (!f.available()) return 0;
  while (f.available()) {
    const int c = f.read();
    if (c < 0) break;
    if (c == '\n') return 1;
    if (out.length() >= maxLen) return -1;
    out += (char)c;
  }
  return 1;   // last line without a trailing newline
}

// Split the legacy "N|data" prefix off a queue line. Lines written before the
// prefix existed (no recognisable prefix) keep the whole line as data.
static void splitPrefix(const String& line, uint8_t& prefix, String& data) {
  prefix = 0;
  data   = line;
  int bar = line.indexOf('|');
  if (bar > 0 && bar <= 3) {
    String p = line.substring(0, bar);
    for (unsigned int k = 0; k < p.length(); k++) {
      if (!isDigit(p[k])) return;
    }
    prefix = (uint8_t)p.toInt();
    data   = line.substring(bar + 1);
  }
}

// The two flushes rewrite the queue through these temp files (see the end of
// each): the kept remainder is written to the temp, the queue removed, and the
// temp renamed over it.
static const char* const PENDING_TMP = "/pending.tmp";    // WiFi flush
static const char* const LORA_TMP    = "/pending.ltmp";   // LoRa flush

// ================================================================
//  recoverInterruptedFlush — heal a crash (brownout, watchdog) between a
//  flush's "remove the queue" and "rename the temp over it": the queue is gone
//  but the temp file holds the kept remainder. Runs before anything counts,
//  appends to or flushes the queue — if a new reading started a fresh queue
//  first, the remainder would be orphaned and the next flush would delete it.
//
//  If the queue still exists, a temp file is the partial copy of a flush that
//  was cut off BEFORE it removed the queue: the queue is authoritative, so the
//  temp is deleted. Left behind, it would be "recovered" the moment the queue
//  next empties, re-queuing readings already delivered (1.2.15).
// ================================================================
static void recoverInterruptedFlush() {
  if (SD.exists(PENDING_FILENAME)) {
    if (SD.exists(PENDING_TMP)) SD.remove(PENDING_TMP);
    if (SD.exists(LORA_TMP))    SD.remove(LORA_TMP);
    return;
  }
  const char* tmp = SD.exists(PENDING_TMP) ? PENDING_TMP
                  : SD.exists(LORA_TMP)    ? LORA_TMP
                  : nullptr;
  if (tmp && SD.rename(tmp, PENDING_FILENAME)) {
    Serial.printf("[Pending] Recovered queue from an interrupted flush (%s)\n", tmp);
  }
}

// ================================================================
//  pendingAppend — append the current reading to /pending.csv (the backlog
//  queue), called when neither LoRa nor a direct upload delivered it. Enforces
//  the cap, dropping the newest reading when full (tidelog.csv on SD keeps the
//  complete record).
// ================================================================
void pendingAppend(const SensorData& data) {
  recoverInterruptedFlush();   // before this append could start a fresh queue

  // Enforce the cap before opening the file for append so we never
  // exceed PENDING_MAX_ENTRIES even when the gateway is offline for
  // an extended period.
  if (pendingCount() >= PENDING_MAX_ENTRIES) {
    // Queue full: drop this (newest) reading rather than the backlog.
    // No data is truly lost — tidelog.csv on SD holds the complete record.
    Serial.printf("[Pending] Queue full (%u) — dropping this reading "
                  "until a flush succeeds\n", PENDING_MAX_ENTRIES);
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
  f.print("0|");   // legacy counter prefix (format compatibility only)
  f.printf("%s,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%u,%d,%d,%.1f,%.1f,%d,%.2f,%d\n",
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
    data.moved         ? 1 : 0,
    data.tiltDeg,
    data.pressureHpa,
    (int)data.sensorType,     // node >= 1.2.10: primary sensor + ultrasonic
    data.distanceUsCm,        //   distance
    data.rtcValid      ? 1 : 0);   // node >= 1.2.11: was the clock trusted?
  f.close();
}

// ================================================================
//  quarantinePendingFile — move a corrupt queue aside instead of choking on it.
//  A torn write or FAT damage can leave a record with no newline; rather than
//  jam the flush forever, rename the file to PENDING_BAD_FILENAME (kept for
//  forensics) and start fresh. tidelog.csv remains the complete record.
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
//  a voltage-scaled per-wake budget), rewriting the kept remainder back to the
//  queue. Returns the number uploaded.
//
//  Failure rule (no outage may cost data): a transport or server failure keeps
//  every entry unchanged and stops the flush; if nothing had been uploaded yet
//  this wake the queue file isn't even rewritten. Only rows the cloud rejects
//  as invalid ("bad" in the batch reply) are dropped.
// ================================================================
uint32_t pendingFlush(float batteryV, uint32_t capOverride) {
  // Heal an interrupted earlier flush BEFORE the budget check, so it is healed
  // even on a wake that then defers.
  recoverInterruptedFlush();

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
    // Without the temp file, deferred entries couldn't be preserved. Abort the
    // whole flush and retry next wake instead of risking the backlog; nothing
    // has been uploaded or deleted yet.
    Serial.println(F("[Pending] Temp open failed — flush deferred"));
    in.close();
    return 0;
  }

  uint32_t uploaded  = 0;
  uint32_t attempted = 0;
  uint32_t kept      = 0;   // failed + over-budget entries preserved for later
  uint32_t dropped   = 0;   // corrupt lines + rows the cloud rejected as invalid
  bool stopAttempts  = false;
  bool sagChecked    = false;  // loaded-voltage re-check done once, after batch 1
  bool abortNoChange = false;  // first batch failed → leave the queue untouched

  String batchRows;
  batchRows.reserve(PENDING_BATCH_MAX_BODY);
  String batchData[PENDING_BATCH_ENTRIES];
  uint8_t batchPrefix[PENDING_BATCH_ENTRIES] = { 0 };
  uint8_t batchCount = 0;

  auto keepLine = [&](uint8_t prefix, const String& data) {
    out.print(prefix); out.print('|'); out.print(data); out.print('\n'); kept++;
  };

  auto flushBatch = [&]() {
    if (batchCount == 0) return;
    esp_task_wdt_reset();
    const bool firstBatch = (attempted == 0);
    uint16_t badRows = 0;
    const bool ok = gasUploadBatch(batchRows, batchCount, badRows);
    attempted += batchCount;
    if (ok) {
      uploaded += batchCount - badRows;
      dropped  += badRows;
      if (badRows > 0) {
        Serial.printf("[Pending] Cloud rejected %u invalid row(s) — dropped\n", badRows);
      }
    } else {
      // Transport/server failure — nothing is wrong with these rows. Keep them
      // exactly as they are and stop for this wake.
      stopAttempts = true;
      if (firstBatch) {
        abortNoChange = true;   // nothing uploaded yet: don't rewrite the file
      } else {
        for (uint8_t i = 0; i < batchCount; i++) keepLine(batchPrefix[i], batchData[i]);
      }
      Serial.println(F("[Pending] Upload failed — backlog kept for the next flush"));
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
    if (!sagChecked && !abortNoChange) {
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

  String line;
  while (!abortNoChange) {
    // Every line does at least an SD read here, even lines that only get
    // deferred (over budget / stopAttempts) below — a large backlog's deferred
    // tail is copied line by line, so feed the watchdog on every line.
    esp_task_wdt_reset();
    const int r = readLineBounded(in, line, PENDING_MAX_LINE_LEN);
    if (r == 0) break;                          // end of file

    // Corruption guard: a valid record is well under PENDING_MAX_LINE_LEN. An
    // oversized "line" means a torn write / FAT damage left a record with no
    // newline — quarantine the whole queue rather than trust the rest of it.
    if (r < 0) {
      Serial.printf("[Pending] Line over %u B — queue corrupt\n",
                    (unsigned)PENDING_MAX_LINE_LEN);
      in.close();
      out.close();
      SD.remove(PENDING_TMP);
      quarantinePendingFile();
      return uploaded;
    }
    line.trim();
    if (line.length() == 0) continue;

    uint8_t prefix;
    String  data;
    splitPrefix(line, prefix, data);

    if (stopAttempts || attempted >= budget) {
      // Over this wake's (voltage-scaled) budget, or a sag/failure stopped the
      // flush — defer to the next flush.
      keepLine(prefix, data);
      continue;
    }

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
      if (abortNoChange) break;
      if (stopAttempts) { keepLine(prefix, data); continue; }
    }
    if (batchCount > 0) batchRows += '\n';
    batchRows += query;
    batchData[batchCount]   = data;
    batchPrefix[batchCount] = prefix;
    batchCount++;

    const uint32_t remainingBudget = budget - attempted;
    if (batchCount >= PENDING_BATCH_ENTRIES || batchCount >= remainingBudget) {
      flushBatch();
      if (abortNoChange) break;
    }
  }
  if (!abortNoChange) flushBatch();
  in.close();
  out.close();

  if (abortNoChange) {
    // The very first upload failed: the queue file is exactly as it was, so
    // just discard the half-written temp copy (no SD rewrite during an outage).
    SD.remove(PENDING_TMP);
    return 0;
  }

  // Replace the queue with the kept remainder (failed + deferred), or clear
  // it. A crash between the remove and the rename is healed by
  // recoverInterruptedFlush() before the queue is next touched.
  SD.remove(PENDING_FILENAME);
  if (kept > 0 && SD.exists(PENDING_TMP)) {
    SD.rename(PENDING_TMP, PENDING_FILENAME);
    Serial.printf("[Pending] Flushed %lu/%lu this wake; %lu still queued, %lu dropped\n",
                  (unsigned long)uploaded, (unsigned long)attempted,
                  (unsigned long)kept, (unsigned long)dropped);
  } else {
    SD.remove(PENDING_TMP);
    Serial.printf("[Pending] Flushed %lu/%lu  queue cleared (%lu dropped)\n",
                  (unsigned long)uploaded, (unsigned long)attempted,
                  (unsigned long)dropped);
  }
  return uploaded;
}

// ================================================================
//  pendingCount — number of queued entries (newline count), 0 if no file.
//
//  Reads in chunks rather than byte-by-byte: at PENDING_MAX_ENTRIES (7,500)
//  the file can be ~750 KB, and a single-byte f.read() per SPI transaction
//  over that many bytes can run long enough to trip the task watchdog with
//  no feed in between — which panics (trigger_panic=true) and reboots the
//  node mid-scan.
// ================================================================
uint32_t pendingCount() {
  recoverInterruptedFlush();   // a queue mid-swap still counts
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

// ================================================================
//  loraFlushPending — drain the pending queue over LoRa (oldest-first) through
//  the gateway, for wakes the WiFi flush didn't handle. Crash-safe via the same
//  temp-file rename pattern as pendingFlush(): a delivered entry is dropped, a
//  NO_ACK stops the flush and the remainder is preserved for the next wake.
//
//  Never drops an entry for failing: an un-ACKed send keeps it unchanged (and,
//  when it's the first send of the wake, leaves the file untouched). An entry
//  too big for one LoRa frame stays queued for the WiFi flush and is skipped
//  here. capOverride (0 = none) is the remote flush_cap.
// ================================================================
uint32_t loraFlushPending(float batteryV, uint32_t capOverride) {
  recoverInterruptedFlush();
  if (!SD.exists(PENDING_FILENAME)) return 0;

  // Same battery gate as the WiFi flush: defer entirely near cutoff so the LoRa
  // TX burst can't brown out a depleted cell (an implausible USB-only reading is
  // not a real cell, so it's never gated).
  if (batteryV > BAT_PLAUSIBLE_MIN_V && batteryV < BAT_FLUSH_LOW_V) {
    Serial.printf("[LoRaFlush] Battery %.2fV low — flush deferred\n", batteryV);
    return 0;
  }

  File in = SD.open(PENDING_FILENAME, FILE_READ);
  if (!in) return 0;
  SD.remove(LORA_TMP);
  File out = SD.open(LORA_TMP, FILE_WRITE);
  if (!out) { in.close(); return 0; }

  uint32_t sent = 0, kept = 0, dropped = 0, skipped = 0;
  bool           stop          = false;    // link down / budget / cap => keep the rest
  bool           abortNoChange = false;    // first send failed → queue untouched
  String         rotated;                  // head entry moved to the tail (see s_loraHeadFails)
  const uint32_t startMs = millis();

  auto keepLine = [&](uint8_t prefix, const String& data) {
    out.print(prefix); out.print('|'); out.print(data); out.print('\n'); kept++;
  };

  String line;
  while (true) {
    esp_task_wdt_reset();
    const int r = readLineBounded(in, line, PENDING_MAX_LINE_LEN);
    if (r == 0) break;

    // Corruption guard (same as pendingFlush): an oversized "line" is a torn
    // write — quarantine the whole queue rather than trust the rest.
    if (r < 0) {
      Serial.printf("[LoRaFlush] Line over %u B — queue corrupt\n",
                    (unsigned)PENDING_MAX_LINE_LEN);
      in.close(); out.close(); SD.remove(LORA_TMP);
      quarantinePendingFile();
      return sent;
    }
    line.trim();
    if (line.length() == 0) continue;

    uint8_t prefix;
    String  data;
    splitPrefix(line, prefix, data);

    // Stopped (a NO_ACK earlier), out of this wake's time budget, or at the
    // remote cap: defer the rest of the queue unchanged to the next wake.
    const bool capHit = (capOverride > 0 && sent >= capOverride);
    if (stop || capHit || (millis() - startMs) > LORA_FLUSH_BUDGET_MS) {
      keepLine(prefix, data);
      continue;
    }

    SensorData d;
    if (!pendingLineToSensorData(data, d)) {
      // Corrupt line — dropping it is the only way past it (tidelog.csv keeps
      // the full record). No TX happened, so it doesn't burn the budget.
      Serial.printf("[LoRaFlush] Dropping corrupt entry: %s\n", data.c_str());
      dropped++;
      continue;
    }

    esp_task_wdt_reset();
    const LoRaSendResult res = loraSend(d, 0);
    esp_task_wdt_reset();

    if (res == LoRaSendResult::OK) {
      sent++;                                // delivered + ACKed => drop from queue
      s_loraHeadFails = 0;
      continue;
    }
    if (res == LoRaSendResult::PAYLOAD_ERROR) {
      // Too big for one LoRa frame: it can't go this way, but the WiFi flush
      // can carry it. Keep it in place and move on to the next entry.
      keepLine(prefix, data);
      skipped++;
      continue;
    }

    // NO_ACK / TX error: the link that just ACKed the live reading dropped
    // this one. Stop sending for this wake; nothing is dropped.
    stop = true;
    if (sent == 0 && dropped == 0) {
      // First send of the wake failed and nothing has changed.
      if (++s_loraHeadFails < LORA_FLUSH_SKIP_AFTER) {
        abortNoChange = true;                // leave the queue file untouched
        break;
      }
      // The same head entry keeps failing while the link is up — move it to
      // the tail so the rest of the backlog isn't stuck behind it.
      s_loraHeadFails = 0;
      rotated = String(prefix) + '|' + data;
      Serial.println(F("[LoRaFlush] Head entry keeps failing — moved to the tail"));
      continue;
    }
    keepLine(prefix, data);
  }

  if (abortNoChange) {
    in.close(); out.close();
    SD.remove(LORA_TMP);
    Serial.printf("[LoRaFlush] No ACK for the first backlog entry (%u/%u) — queue unchanged\n",
                  (unsigned)s_loraHeadFails, (unsigned)LORA_FLUSH_SKIP_AFTER);
    return 0;
  }
  if (rotated.length() > 0) { out.print(rotated); out.print('\n'); kept++; }
  in.close();
  out.close();

  // Nothing sent, dropped or moved (e.g. every entry was too big for LoRa):
  // the queue is unchanged, so skip the rewrite.
  if (sent == 0 && dropped == 0 && rotated.length() == 0) {
    SD.remove(LORA_TMP);
    return 0;
  }

  // Replace the queue with the kept remainder, or clear it. A crash between the
  // remove and the rename is healed by recoverInterruptedFlush() next wake.
  SD.remove(PENDING_FILENAME);
  if (kept > 0 && SD.exists(LORA_TMP)) {
    SD.rename(LORA_TMP, PENDING_FILENAME);
  } else {
    SD.remove(LORA_TMP);
  }
  if (sent > 0 || kept > 0 || dropped > 0) {
    Serial.printf("[LoRaFlush] Sent %lu over LoRa; %lu still queued (%lu too big for LoRa), %lu dropped\n",
                  (unsigned long)sent, (unsigned long)kept, (unsigned long)skipped,
                  (unsigned long)dropped);
  }
  return sent;
}
