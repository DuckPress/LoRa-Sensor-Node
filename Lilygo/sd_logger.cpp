#include "sd_logger.h"
#include "config.h"
#include <SD.h>
#include <SPI.h>

static bool s_sdReady = false;

// ================================================================
//  sdInit — mount the MicroSD on the shared FSPI bus, create /tidelog.csv with
//  its header if missing, and confirm it is appendable. Retries the mount once.
//  Sets the internal ready flag that gates sdLog()/sdReady().
// ================================================================
void sdInit() {
  // Uses the default SPI instance initialised in setup() (SD bus, FSPI).
  if (!SD.begin(PIN_SD_CS, SPI)) {
    SD.end();
    delay(200);
    if (!SD.begin(PIN_SD_CS, SPI)) {
      Serial.println(F("[SD] Mount FAILED after retry — check card and wiring"));
      s_sdReady = false;
      return;
    }
  }

  Serial.printf("[SD] Mounted OK — %.1f MB\n", SD.cardSize() / 1048576.0);

  // Schema-drift guard: if an existing tidelog.csv was written by a firmware
  // with a DIFFERENT column layout, its header won't match LOG_HEADER. Appending
  // new-format rows under an old header produces one file with mixed schemas
  // that every downstream parser (tide_reduce.html included) silently misreads
  // — exactly what happened on the field card (7/10/14-column rows in one file).
  // Roll the mismatched file aside to a single backup and start a clean one so
  // every tidelog.csv is internally consistent from here on.
  if (SD.exists(LOG_FILENAME)) {
    File hf = SD.open(LOG_FILENAME, FILE_READ);
    if (hf) {
      String existing = hf.readStringUntil('\n');
      hf.close();
      existing.trim();
      String want = String(LOG_HEADER);
      want.trim();
      if (existing != want) {
        Serial.println(F("[SD] tidelog.csv header differs from firmware schema "
                         "— rolling old file to /tidelog_old.csv"));
        SD.remove(LOG_ROLLED_FILENAME);          // keep only the latest backup
        if (!SD.rename(LOG_FILENAME, LOG_ROLLED_FILENAME)) {
          Serial.println(F("[SD] WARNING: could not roll old log — leaving as-is"));
        }
      }
    }
  }

  if (!SD.exists(LOG_FILENAME)) {
    File f = SD.open(LOG_FILENAME, FILE_WRITE);
    if (!f) {
      // Falling through would let the append probe below CREATE the file
      // (FILE_APPEND creates missing files) — headerless, silently corrupting
      // the CSV. Treat the card as not ready instead.
      Serial.println(F("[SD] Could not create log file — card not ready"));
      s_sdReady = false;
      return;
    }
    f.print(LOG_HEADER);
    f.close();
    Serial.printf("[SD] Created %s\n", LOG_FILENAME);
  }

  // Verify append access before declaring ready
  File probe = SD.open(LOG_FILENAME, FILE_APPEND);
  if (!probe) {
    Serial.println(F("[SD] Log file not appendable — reformat card as FAT32"));
    s_sdReady = false;
    return;
  }
  probe.close();
  s_sdReady = true;
  Serial.println(F("[SD] Log file OK"));
}

// ================================================================
//  sdLog — append one reading as a CSV row to /tidelog.csv (the node's complete
//  record). Marks the card not-ready on any failure so later calls fail fast.
//  Returns true only on a successful append.
// ================================================================
bool sdLog(const SensorData& data, uint32_t wakeCount, int32_t wifiRssi) {
  if (!s_sdReady) return false;

  File f = SD.open(LOG_FILENAME, FILE_APPEND);
  if (!f) {
    Serial.println(F("[SD] ERROR: failed to open log for append"));
    s_sdReady = false;
    return false;
  }

  // CSV columns match LOG_HEADER in config.h:
  // timestamp, rtc_valid, dist_raw_cm, dist_kalman_cm, water_level_cm,
  // temp_c, humidity_pct, bat_v, wifi_rssi, wake_count,
  // burst_sd_cm, burst_n, survey_mode, mount_moved
  //
  // Format the row first so a SHORT write (card full mid-row) is detected —
  // comparing only against 0 would call a truncated row a success.
  char row[232];
  int len = snprintf(row, sizeof(row),
    "%s,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%ld,%lu,%.2f,%u,%d,%d\n",
    data.isoTimestamp,
    data.rtcValid      ? 1    : 0,
    data.distanceValid ? data.distanceRaw  : -1.0f,
    data.distanceValid ? data.distanceCm   : -1.0f,
    data.distanceValid ? data.waterLevelCm : -1.0f,
    data.envValid      ? data.tempC        : -999.0f,
    data.envValid      ? data.humidity     : -1.0f,
    data.batValid      ? data.batV         : 0.0f,
    (long)wifiRssi,
    (unsigned long)wakeCount,
    data.burstSd,
    (unsigned)data.burstN,
    data.surveyMode    ? 1 : 0,
    data.moved         ? 1 : 0);
  if (len <= 0 || (size_t)len >= sizeof(row)) {
    f.close();
    Serial.println(F("[SD] Row format failed"));
    return false;
  }

  size_t written = f.print(row);
  f.close();

  if (written != (size_t)len) {
    Serial.printf("[SD] Short write (%u/%d bytes) — card full?\n",
                  (unsigned)written, len);
    s_sdReady = false;
    return false;
  }
  return true;
}

// True once the card mounted and the log file was confirmed writable.
bool sdReady() { return s_sdReady; }

// ================================================================
//  sdLogBootEvent — append one row to /bootlog.csv for a real reboot
//  (brownout/panic/power-on — NOT a deep-sleep wake), creating the file with a
//  header on first use. Makes unattended crashes visible after the fact.
// ================================================================
bool sdLogBootEvent(const char* reason, uint32_t wakeCount, const char* isoTs) {
  if (!s_sdReady) return false;

  bool fresh = !SD.exists(BOOTLOG_FILENAME);
  File f = SD.open(BOOTLOG_FILENAME, FILE_APPEND);
  if (!f) {
    Serial.println(F("[SD] bootlog open failed"));
    return false;
  }
  if (fresh) f.print(F("timestamp,wake_count,reset_reason\n"));
  f.printf("%s,%lu,%s\n", isoTs, (unsigned long)wakeCount, reason);
  f.close();
  Serial.printf("[Boot] logged reset '%s' to %s\n", reason, BOOTLOG_FILENAME);
  return true;
}
