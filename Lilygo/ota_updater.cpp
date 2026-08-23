#include "ota_updater.h"
#include "config.h"
#include "net_tls.h"
#include "wifi_manager.h"
#include "display_mgr.h"

#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <string.h>

static bool s_initDone = false;

// A parsed "MAJOR.MINOR.PATCH" version, used to compare the local build against
// the version published in the GitHub release's version.txt.
struct SemanticVersion {
  uint16_t major = 0, minor = 0, patch = 0;
};

// Parse a bare "X.Y.Z" string (no leading 'v') into out. Returns false unless
// it is exactly three non-negative integers.
static bool parseVersion(const char* str, SemanticVersion& out) {
  int ma = 0, mi = 0, pa = 0;
  char tail = 0;
  // The trailing %c must NOT match: it catches junk after the patch number
  // ("1.2.3-beta", "1.2.30extra"), which would otherwise be silently accepted.
  if (sscanf(str, "%d.%d.%d%c", &ma, &mi, &pa, &tail) != 3) return false;
  if (ma < 0 || mi < 0 || pa < 0 ||
      ma > 65535 || mi > 65535 || pa > 65535) return false;
  out.major = (uint16_t)ma;
  out.minor = (uint16_t)mi;
  out.patch = (uint16_t)pa;
  return true;
}

// True only if `remote` is strictly newer than `local` (major, then minor, then
// patch). Equal/older returns false, so OTA never re-flashes or downgrades.
static bool isNewer(const SemanticVersion& remote, const SemanticVersion& local) {
  if (remote.major != local.major) return remote.major > local.major;
  if (remote.minor != local.minor) return remote.minor > local.minor;
  return remote.patch > local.patch;
}

// HTTPS-GET the release's version.txt from GitHub and copy its trimmed body
// into buf. TLS-validated against the CA bundle. Returns false on any HTTP
// error or an empty body.
static bool fetchVersionString(char* buf, size_t bufSize) {
  WiFiClientSecure client;
  applyTls(client);   // validate GitHub's cert — see net_tls.h

  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setTimeout(OTA_HTTP_TIMEOUT_MS);

  char url[256];
  snprintf(url, sizeof(url),
           "https://" OTA_GITHUB_HOST
           "/" OTA_GITHUB_USER
           "/" OTA_GITHUB_REPO
           "/releases/latest/download/" OTA_VERSION_FILENAME);

  Serial.printf("[OTA] Checking: %s\n", url);

  if (!http.begin(client, url)) {
    Serial.println(F("[OTA] http.begin() failed"));
    return false;
  }

  int code = http.GET();
  if (code != 200) {
    Serial.printf("[OTA] version.txt HTTP %d\n", code);
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();
  body.trim();

  if (body.isEmpty()) {
    Serial.println(F("[OTA] version.txt empty"));
    return false;
  }

  strncpy(buf, body.c_str(), bufSize - 1);
  buf[bufSize - 1] = '\0';
  return true;
}

// HTTPUpdate progress callback: redraw the OLED progress bar and feed the
// watchdog so the multi-second firmware download can't trip a WDT reset.
static void onOTAProgress(int current, int total) {
  esp_task_wdt_reset();   // feed FIRST — even when the size is unknown
  if (total <= 0) return; // chunked download: no total → no progress bar
  // Redraw only when the percentage actually changes; a full OLED update on
  // every chunk measurably slows the download.
  static int lastPct = -1;
  int pct = (int)((long)current * 100L / total);
  if (pct != lastPct) {
    displayOTAProgress(pct);
    lastPct = pct;
  }
}

void otaInit() {
  if (s_initDone) return;
  httpUpdate.onProgress(onOTAProgress);
  httpUpdate.rebootOnUpdate(true);
  // GitHub's release-asset download URL 302-redirects; without this the
  // firmware.bin fetch dies on the redirect on some core versions.
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  s_initDone = true;
  Serial.println(F("[OTA] Initialised — firmware: " FIRMWARE_VERSION));
}

bool checkForOTAUpdate() {
  if (strcmp(OTA_GITHUB_USER, "YOUR_GITHUB_USERNAME") == 0) {
    Serial.println(F("[OTA] OTA_GITHUB_USER not configured — skipping"));
    return false;
  }

  if (wifiStatus() != WL_CONNECTED) {
    Serial.println(F("[OTA] No WiFi — skipping"));
    return false;
  }

  char remoteStr[32] = { 0 };
  if (!fetchVersionString(remoteStr, sizeof(remoteStr))) return false;
  Serial.printf("[OTA] Remote: %s  Local: %s\n", remoteStr, FIRMWARE_VERSION);

  SemanticVersion remote, local;
  if (!parseVersion(remoteStr,    remote)) return false;
  if (!parseVersion(FIRMWARE_VERSION, local)) return false;

  if (!isNewer(remote, local)) {
    Serial.println(F("[OTA] Already up to date"));
    return false;
  }

  // Refuse to re-attempt a version we already flashed once: if version.txt
  // says X but the published firmware.bin still reports an older
  // FIRMWARE_VERSION, the node would otherwise reflash the same image on
  // every OTA wake forever. NVS survives both reboots and power loss.
  Preferences prefs;
  prefs.begin("ota", false);
  String lastTry = prefs.getString("lastTry", "");
  if (lastTry == remoteStr) {
    prefs.end();
    Serial.printf("[OTA] %s was already attempted — skipping. Does the "
                  "binary's FIRMWARE_VERSION match version.txt?\n", remoteStr);
    return false;
  }
  prefs.putString("lastTry", remoteStr);
  prefs.end();

  Serial.printf("[OTA] Updating %s → %s\n", FIRMWARE_VERSION, remoteStr);
  displayOTAProgress(0);

  char binUrl[256];
  snprintf(binUrl, sizeof(binUrl),
           "https://" OTA_GITHUB_HOST
           "/" OTA_GITHUB_USER
           "/" OTA_GITHUB_REPO
           "/releases/latest/download/" OTA_BIN_FILENAME);

  // Validate the TLS chain before flashing — an unauthenticated download here
  // would let a network attacker push malicious firmware (see net_tls.h).
  WiFiClientSecure client;
  applyTls(client);

  esp_task_wdt_reset();
  t_httpUpdate_return ret = httpUpdate.update(client, binUrl);
  esp_task_wdt_reset();

  switch (ret) {
    case HTTP_UPDATE_OK:
      Serial.println(F("[OTA] Success — rebooting"));
      delay(1000);
      ESP.restart();
      return true;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(F("[OTA] No update (unexpected)"));
      return false;
    default:
      Serial.printf("[OTA] FAILED (%d): %s\n",
                    httpUpdate.getLastError(),
                    httpUpdate.getLastErrorString().c_str());
      // Transient failure (network/TLS hiccup) — clear the marker so this
      // version can be retried on the next OTA wake.
      prefs.begin("ota", false);
      prefs.remove("lastTry");
      prefs.end();
      return false;
  }
}
