#include "wifi_manager.h"
#include "config.h"
#include "secret_store.h"   // WiFi creds now come from the SD card, not the binary
#include <esp_task_wdt.h>
#include <string.h>

static int8_t s_connectedIdx = -1;

// ---- Fast-reconnect cache (survives deep sleep in RTC RAM) ------------------
// A full WiFi.begin(ssid, pass) scans every channel before associating, which
// is the single biggest radio-on cost of a WiFi wake. After a successful
// connect we remember which network it was and its exact channel + BSSID, so
// the next wake can associate directly (no scan) in well under a second.
RTC_DATA_ATTR static bool    s_cacheValid     = false;
RTC_DATA_ATTR static int8_t  s_cacheIdx       = -1;
RTC_DATA_ATTR static uint8_t s_cacheBssid[6]  = { 0 };
RTC_DATA_ATTR static uint8_t s_cacheChannel   = 0;

// Cache the AP we just associated with (index + channel + BSSID) into RTC RAM
// so the next wake can skip the all-channel scan via fastReconnect().
static void rememberConnection(int8_t idx) {
  s_connectedIdx = idx;
  s_cacheIdx     = idx;
  s_cacheChannel = WiFi.channel();
  const uint8_t* bssid = WiFi.BSSID();
  if (bssid) memcpy(s_cacheBssid, bssid, 6);
  s_cacheValid = (bssid != nullptr);
}

// Targeted, scan-free association to the last-known AP. Returns true on success.
static bool fastReconnect() {
  if (!s_cacheValid || s_cacheIdx < 0 || s_cacheIdx >= (int)secretWifiCount()) return false;

  const char* ssid = secretWifiSsid(s_cacheIdx);
  const char* pass = secretWifiPass(s_cacheIdx);
  Serial.printf("[WiFi] Fast reconnect to %s (ch %u)\n", ssid, s_cacheChannel);
  WiFi.begin(ssid, pass, s_cacheChannel, s_cacheBssid);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_FAST_TIMEOUT_MS) {
    delay(100);
    esp_task_wdt_reset();
  }

  if (WiFi.status() == WL_CONNECTED) {
    rememberConnection(s_cacheIdx);
    Serial.printf("[WiFi] Connected (fast) to %s  IP=%s  RSSI=%d dBm\n",
                  ssid, WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }

  Serial.println(F("[WiFi] Fast reconnect failed — falling back to scan"));
  s_cacheValid = false;          // stale cache — don't try it again this wake
  WiFi.disconnect(true);
  delay(200);
  return false;
}

// ================================================================
//  wifiConnect — associate with WiFi: try the cached AP first (scan-free), then
//  fall back to scanning each SSID from the secret store in order. Caches the
//  winning AP for next wake. Returns true once connected.
// ================================================================
bool wifiConnect() {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  // Modem sleep lets WiFi.RSSI() read back stale — often exactly 0 — between
  // beacon intervals. This wake's connection is short-lived (a few seconds),
  // so the power saved by sleep isn't worth losing valid RSSI telemetry.
  WiFi.setSleep(false);

  // Try the cached AP first; it succeeds on the common steady-state path.
  if (fastReconnect()) return true;

  WiFi.disconnect(true);
  delay(200);

  const uint8_t nNets = secretWifiCount();
  if (nNets == 0) {
    Serial.println(F("[WiFi] No WiFi networks provisioned (see /secrets.txt)"));
  }
  for (uint8_t i = 0; i < nNets; i++) {
    const char* ssid = secretWifiSsid(i);
    const char* pass = secretWifiPass(i);
    Serial.printf("[WiFi] Trying SSID %d: %s\n", i + 1, ssid);

    WiFi.begin(ssid, pass);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
      Serial.print('.');
      esp_task_wdt_reset();  // connect loop is the longest blocking stretch
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      rememberConnection(i);     // cache channel + BSSID for next wake
      Serial.printf("[WiFi] Connected to %s  IP=%s  RSSI=%d dBm\n",
                    ssid,
                    WiFi.localIP().toString().c_str(),
                    WiFi.RSSI());
      return true;
    }

    WiFi.disconnect(true);
    delay(500);
  }

  s_connectedIdx = -1;
  Serial.println(F("[WiFi] All SSIDs failed"));
  return false;
}

wl_status_t wifiStatus()       { return WiFi.status(); }
int8_t      wifiCurrentIndex() { return s_connectedIdx; }
int32_t     wifiRSSI()         { return (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0; }
