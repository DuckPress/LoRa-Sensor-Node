#include "crashlog.h"
#include "config.h"
#include "sd_logger.h"
#include "gas_upload.h"
#include <Preferences.h>
#include <esp_attr.h>
#include <esp_core_dump.h>

// ---- Stage breadcrumb (RTC_NOINIT survives panics / software resets) --------
static RTC_NOINIT_ATTR uint32_t s_crumbMagic;
static RTC_NOINIT_ATTR uint8_t  s_crumbStage;
static constexpr uint32_t CRUMB_MAGIC = 0x5AFEC0DEUL;

static uint8_t s_prevStage  = STAGE_NONE;   // what the previous run was doing
static char    s_reasonStr[12] = "UNKNOWN";  // this boot's reset reason (short)

static const char* const STAGE_NAMES[STAGE__COUNT] = {
  "NONE", "BOOT", "SENSORS_INIT", "SD_INIT", "SECRETS", "LORA_INIT",
  "READ_SENSORS", "SD_LOG", "LORA_TX", "WIFI_CONNECT", "NTP", "NODECFG",
  "OTA", "WIFI_FLUSH", "UPLOAD", "LORA_FLUSH", "DISPLAY", "SLEEP",
};

const char* crashStageName(uint8_t stage) {
  return (stage < STAGE__COUNT) ? STAGE_NAMES[stage] : "?";
}

void crashStageSet(uint8_t stage) {
  s_crumbMagic = CRUMB_MAGIC;
  s_crumbStage = stage;
}

static const char* reasonShort(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

uint8_t crashBootBegin(esp_reset_reason_t reason) {
  s_prevStage = (s_crumbMagic == CRUMB_MAGIC && s_crumbStage < STAGE__COUNT)
                ? s_crumbStage : (uint8_t)STAGE_NONE;
  strncpy(s_reasonStr, reasonShort(reason), sizeof(s_reasonStr) - 1);
  s_reasonStr[sizeof(s_reasonStr) - 1] = '\0';
  crashStageSet(STAGE_BOOT);
  if (reason != ESP_RST_DEEPSLEEP && reason != ESP_RST_POWERON) {
    Serial.printf("[Crash] Previous run was in stage %s when it reset (%s)\n",
                  crashStageName(s_prevStage), s_reasonStr);
  }
  return s_prevStage;
}

// ---- Core dump summary -----------------------------------------------------
bool crashDumpPresent() {
  return esp_core_dump_image_check() == ESP_OK;
}

// Human name for the common Xtensa exception causes (the rest stay numeric).
static const char* excCauseName(uint32_t c) {
  switch (c) {
    case 0:  return "IllegalInstruction";
    case 2:  return "InstrFetchError";
    case 3:  return "LoadStoreError";
    case 6:  return "DivideByZero";
    case 9:  return "LoadStoreAlignment";
    case 20: return "InstrFetchProhibited";
    case 28: return "LoadProhibited";
    case 29: return "StoreProhibited";
    default: return "";
  }
}

// Only [A-Za-z0-9_-] survives — task names go into a URL and a CSV.
static void sanitize(char* s) {
  for (; *s; s++) {
    if (!isalnum((unsigned char)*s) && *s != '_' && *s != '-') *s = '_';
  }
}

// Fill the summary; false if no dump / unreadable.
static bool getSummary(esp_core_dump_summary_t& sum) {
  memset(&sum, 0, sizeof(sum));
  if (!crashDumpPresent()) return false;
  return esp_core_dump_get_summary(&sum) == ESP_OK;
}

// Backtrace as "pc1,pc2,..." (hex, no 0x) into buf.
static void formatBacktrace(const esp_core_dump_summary_t& sum, char* buf, size_t n) {
  size_t used = 0;
  uint32_t depth = sum.exc_bt_info.depth;
  if (depth > 16) depth = 16;
  buf[0] = '\0';
  for (uint32_t i = 0; i < depth && used + 10 < n; i++) {
    int w = snprintf(buf + used, n - used, "%s%08lx", i ? "," : "",
                     (unsigned long)sum.exc_bt_info.bt[i]);
    if (w <= 0) break;
    used += (size_t)w;
  }
}

static bool buildQuery(char* out, size_t outSize, const char* isoTs) {
  static esp_core_dump_summary_t sum;   // ~300 B — keep it off the stack
  if (!getSummary(sum)) return false;
  char task[17]; strncpy(task, sum.exc_task, 16); task[16] = '\0'; sanitize(task);
  char bt[16 * 9 + 1]; formatBacktrace(sum, bt, sizeof(bt));
  char sha[17]; strncpy(sha, (const char*)sum.app_elf_sha256, 16); sha[16] = '\0'; sanitize(sha);
  int n = snprintf(out, outSize,
    "action=crash&id=%u&fw=%s&ts=%s&reason=%s&stage=%s"
    "&exc=%lu&excn=%s&task=%s&pc=%08lx&vaddr=%08lx&bt=%s&depth=%lu&corrupt=%d&sha=%s",
    (unsigned)NODE_ID, FIRMWARE_VERSION, isoTs ? isoTs : "",
    s_reasonStr, crashStageName(s_prevStage),
    (unsigned long)sum.ex_info.exc_cause, excCauseName(sum.ex_info.exc_cause),
    task, (unsigned long)sum.exc_pc, (unsigned long)sum.ex_info.exc_vaddr,
    bt, (unsigned long)sum.exc_bt_info.depth, sum.exc_bt_info.corrupted ? 1 : 0, sha);
  return n > 0 && (size_t)n < outSize;
}

// ---- SD log (once per dump) ------------------------------------------------
void crashLogToSdOnce(const char* isoTs, uint32_t wakeCount) {
  if (!crashDumpPresent()) return;
  Preferences p;
  p.begin("crash", false);
  bool logged = p.getUChar("sdlog", 0) != 0;
  if (!logged) {
    static esp_core_dump_summary_t sum;
    if (getSummary(sum)) {
      char task[17]; strncpy(task, sum.exc_task, 16); task[16] = '\0'; sanitize(task);
      char bt[16 * 9 + 1]; formatBacktrace(sum, bt, sizeof(bt));
      for (char* c = bt; *c; c++) if (*c == ',') *c = ' ';   // CSV-safe
      char line[320];
      snprintf(line, sizeof(line), "%s,%s,%lu,%s,%s,%s,%08lx,%s,%.16s",
               s_reasonStr, crashStageName(s_prevStage),
               (unsigned long)sum.ex_info.exc_cause, excCauseName(sum.ex_info.exc_cause),
               task, sum.exc_bt_info.corrupted ? "corrupt" : "ok",
               (unsigned long)sum.exc_pc, bt, (const char*)sum.app_elf_sha256);
      if (sdLogCrashEvent(isoTs, wakeCount, line)) p.putUChar("sdlog", 1);
      Serial.printf("[Crash] Core dump found: %s in %s at pc=%08lx (%s), %lu frames\n",
                    s_reasonStr, task, (unsigned long)sum.exc_pc,
                    excCauseName(sum.ex_info.exc_cause),
                    (unsigned long)sum.exc_bt_info.depth);
    }
  }
  p.end();
}

// ---- Upload (WiFi wake) ----------------------------------------------------
bool crashUploadIfAny(const char* isoTs) {
  if (!crashDumpPresent()) return true;
  char q[640];
  if (!buildQuery(q, sizeof(q), isoTs)) return false;
  String body;
  if (!gasFetch(q, body)) {
    Serial.println(F("[Crash] Upload failed — dump kept for the next WiFi wake"));
    return false;
  }
  esp_core_dump_image_erase();
  Preferences p;
  p.begin("crash", false);
  p.putUChar("sdlog", 0);
  p.end();
  Serial.println(F("[Crash] Summary uploaded — dump erased"));
  return true;
}
