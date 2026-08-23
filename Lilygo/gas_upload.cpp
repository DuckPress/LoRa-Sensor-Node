#include "gas_upload.h"
#include "config.h"
#include "net_tls.h"
#include "secret_store.h"   // GAS host/path now come from the SD card, not the binary
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// Core transport: one query string to the GAS web app over HTTPS GET
// (TLS-validated, following the GAS 302 redirect), body returned to the
// caller. Centralised so every path — uploads, pending flush, config fetch —
// shares identical URL/TLS/redirect handling and can't drift apart.
bool gasFetch(const char* query, String& bodyOut) {
  bodyOut = "";
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[Upload] No WiFi — skipping"));
    return false;
  }

  // WHY GET (not POST): GAS always answers with a 302 redirect. HTTPClient
  // builds a fresh TLS client for the redirected leg, which would drop a POST
  // body; GET query parameters survive the redirect intact.
  // Recovery batches carry eight readings as URL-safe base64. Keep this below
  // normal web-server URL limits while sharing the same TLS/redirect code.
  const char* gasHost = secretGasHost();
  const char* gasPath = secretGasPath();
  if (gasHost[0] == '\0' || gasPath[0] == '\0') {
    Serial.println(F("[Upload] GAS endpoint not provisioned (see /secrets.txt)"));
    return false;
  }
  char url[2304];
  int n = snprintf(url, sizeof(url), "https://%s%s?%s", gasHost, gasPath, query);
  if (n <= 0 || (size_t)n >= sizeof(url)) {
    Serial.println(F("[Upload] URL too long — dropping"));
    return false;
  }

  WiFiClientSecure client;
  applyTls(client);

  HTTPClient http;
  if (!http.begin(client, url)) {
    Serial.println(F("[Upload] http.begin() failed"));
    return false;
  }
  http.setTimeout(UPLOAD_HTTP_TIMEOUT_MS);
  // FORCE keeps the same (validated) TLS client across the GAS 302 redirect.
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  int code = http.GET();
  Serial.printf("[Upload] HTTP %d\n", code);
  if (code < 0) {
    Serial.printf("[Upload] Error: %s\n", http.errorToString(code).c_str());
    http.end();
    return false;
  }

  // GAS answers HTTP 200 even for application-level failures (lock timeout,
  // auth rejection) — the body is {"ok":false,...}. Only a 200 whose body
  // reports ok:true counts as delivered; otherwise the pending queue would
  // silently drop entries that never reached the sheet.
  bodyOut = http.getString();
  http.end();

  // Anchored at the start rather than indexOf() anywhere in the body: every
  // apps_script.gs response is built by json({ok:...,...}), so "ok" is always
  // the first serialised key. A plain substring search would also match
  // "ok":true appearing inside an echoed/error field further into the body.
  bool ok = (code == 200) && bodyOut.startsWith("{\"ok\":true");
  if (code == 200 && !ok) {
    Serial.printf("[Upload] GAS rejected: %s\n", bodyOut.c_str());
  }
  return ok;
}

// Upload-only wrapper: same transport, body discarded.
bool gasUpload(const char* query) {
  String body;
  return gasFetch(query, body);
}

// URL-safe base64 avoids '&', '+' and '/' being interpreted as query syntax.
static String base64UrlEncode(const String& input) {
  static const char alphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
  String out;
  out.reserve(((input.length() + 2) / 3) * 4);
  const uint8_t* src = (const uint8_t*)input.c_str();
  const size_t len = input.length();
  for (size_t i = 0; i < len; i += 3) {
    uint32_t v = (uint32_t)src[i] << 16;
    const bool hasSecond = (i + 1 < len);
    const bool hasThird  = (i + 2 < len);
    if (hasSecond) v |= (uint32_t)src[i + 1] << 8;
    if (hasThird)  v |= src[i + 2];
    out += alphabet[(v >> 18) & 0x3F];
    out += alphabet[(v >> 12) & 0x3F];
    if (hasSecond) out += alphabet[(v >> 6) & 0x3F];
    if (hasThird)  out += alphabet[v & 0x3F];
  }
  return out;
}

static int jsonUint(const String& body, const char* key) {
  String needle = String('"') + key + "\":";
  int start = body.indexOf(needle);
  if (start < 0) return -1;
  start += needle.length();
  int end = start;
  while (end < (int)body.length() && isDigit(body[end])) end++;
  return (end == start) ? -1 : body.substring(start, end).toInt();
}

bool gasUploadBatch(const String& rows, uint16_t expectedRows, uint16_t& badRows) {
  badRows = 0;
  if (expectedRows == 0 || rows.length() == 0) return false;

  String query = "action=batch&b=" + base64UrlEncode(rows);
  String body;
  if (!gasFetch(query.c_str(), body)) return false;

  const int written = jsonUint(body, "written");
  const int dup     = jsonUint(body, "dup");
  const int bad     = jsonUint(body, "bad");
  const int authBad = jsonUint(body, "auth_bad");
  if (written < 0 || dup < 0 || bad < 0 || authBad < 0 || authBad > 0 ||
      written + dup + bad != expectedRows) {
    Serial.printf("[Upload] Invalid batch response: %s\n", body.c_str());
    return false;
  }
  badRows = (uint16_t)bad;
  return true;
}
