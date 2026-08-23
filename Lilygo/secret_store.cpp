#include "secret_store.h"
#include "config.h"
#include <SD.h>
#include <Preferences.h>

// ================================================================
//  Parsed secret store (fixed buffers — no dynamic String kept around, so
//  the accessors hand out stable pointers for the whole wake).
// ================================================================
static struct {
  char    ssid[SECRET_MAX_WIFI][33];   // WiFi SSID  (max 32 chars + NUL)
  char    pass[SECRET_MAX_WIFI][65];   // WPA2 PSK   (max 63 chars + NUL, plus slack)
  uint8_t wifiCount;
  char    gasHost[64];
  char    gasPath[200];
  char    loraToken[33];
  bool    valid;
} s_sec;

// Copy src into dst, NUL-terminated, never overflowing dstSize.
static void copyField(char* dst, size_t dstSize, const String& src) {
  size_t n = src.length();
  if (n >= dstSize) n = dstSize - 1;
  memcpy(dst, src.c_str(), n);
  dst[n] = '\0';
}

// Parse the whole secrets file text into s_sec. Returns true if it yielded at
// least a usable shape (a GAS host+path; WiFi networks are strongly expected
// but a token-only LoRa node is still allowed to come up).
static bool parseSecrets(const String& text) {
  memset(&s_sec, 0, sizeof(s_sec));

  int pos = 0;
  const int len = text.length();
  while (pos < len) {
    int nl = text.indexOf('\n', pos);
    if (nl < 0) nl = len;
    String line = text.substring(pos, nl);
    pos = nl + 1;

    line.trim();
    if (line.length() == 0 || line[0] == '#') continue;   // blank / comment

    int eq = line.indexOf('=');
    if (eq < 0) continue;
    String key = line.substring(0, eq);   key.trim();
    String val = line.substring(eq + 1);  val.trim();

    // Strip a trailing inline comment on non-wifi values (a '#' after the value)
    // — kept simple: only when preceded by whitespace so a '#' inside a password
    // isn't clipped. (WiFi lines are taken verbatim; passwords may contain '#'.)
    if (key != "wifi") {
      int hash = val.indexOf(" #");
      if (hash >= 0) { val = val.substring(0, hash); val.trim(); }
    }

    if (key == "wifi") {
      int comma = val.indexOf(',');
      if (comma < 0) continue;                 // malformed — needs SSID,PASS
      String ssid = val.substring(0, comma);   ssid.trim();
      String pass = val.substring(comma + 1);  // NOT trimmed: passwords may end in spaces? keep as-is minus edges
      pass.trim();
      if (ssid.length() == 0 || s_sec.wifiCount >= SECRET_MAX_WIFI) continue;
      copyField(s_sec.ssid[s_sec.wifiCount], sizeof(s_sec.ssid[0]), ssid);
      copyField(s_sec.pass[s_sec.wifiCount], sizeof(s_sec.pass[0]), pass);
      s_sec.wifiCount++;
    } else if (key == "gas_host") {
      copyField(s_sec.gasHost, sizeof(s_sec.gasHost), val);
    } else if (key == "gas_path") {
      copyField(s_sec.gasPath, sizeof(s_sec.gasPath), val);
    } else if (key == "lora_token") {
      copyField(s_sec.loraToken, sizeof(s_sec.loraToken), val);
    }
  }

  // "Usable" = we can at least reach the cloud. A node with WiFi networks but
  // no GAS path can't upload; a node with a GAS path but no WiFi can't connect.
  // Require both for validity; the LoRa token is legitimately allowed to be "".
  s_sec.valid = (s_sec.wifiCount > 0) &&
                (s_sec.gasHost[0] != '\0') && (s_sec.gasPath[0] != '\0');
  return s_sec.valid;
}

// Read the whole /secrets.txt from SD into `out`. Returns false if absent/empty.
static bool readSecretsFromSD(String& out) {
  if (!SD.exists(SECRETS_FILENAME)) return false;
  File f = SD.open(SECRETS_FILENAME, FILE_READ);
  if (!f) return false;
  // Cap the read so a corrupt/huge file can't exhaust RAM.
  const size_t MAX = 4096;
  out = "";
  out.reserve(f.size() < MAX ? f.size() + 1 : MAX);
  while (f.available() && out.length() < MAX) out += (char)f.read();
  f.close();
  return out.length() > 0;
}

bool secretsLoad() {
  Preferences prefs;
  String text;

  // 1) Prefer the SD file: it's how the operator provisions/edits credentials.
  if (readSecretsFromSD(text) && parseSecrets(text)) {
    // Refresh the NVS cache so the node keeps working if the card is later
    // removed or a future read glitches.
    prefs.begin("secrets", false);
    prefs.putString("raw", text);
    prefs.end();
    Serial.printf("[Secrets] Loaded from SD (%u WiFi network(s))\n", s_sec.wifiCount);
    return true;
  }

  // 2) Fall back to the NVS cache from a previous successful SD load.
  prefs.begin("secrets", true);   // read-only
  String cached = prefs.getString("raw", "");
  prefs.end();
  if (cached.length() > 0 && parseSecrets(cached)) {
    Serial.printf("[Secrets] SD file missing/invalid — using NVS cache "
                  "(%u WiFi network(s))\n", s_sec.wifiCount);
    return true;
  }

  Serial.println(F("[Secrets] NONE found — add /secrets.txt to the SD card "
                   "(see secrets.example.txt). WiFi/GAS/LoRa-token unavailable."));
  s_sec.valid = false;
  return false;
}

bool        secretsAvailable()            { return s_sec.valid; }
uint8_t     secretWifiCount()             { return s_sec.wifiCount; }
const char* secretWifiSsid(uint8_t i)     { return (i < s_sec.wifiCount) ? s_sec.ssid[i] : ""; }
const char* secretWifiPass(uint8_t i)     { return (i < s_sec.wifiCount) ? s_sec.pass[i] : ""; }
const char* secretGasHost()               { return s_sec.gasHost; }
const char* secretGasPath()               { return s_sec.gasPath; }
const char* secretLoraToken()             { return s_sec.loraToken; }
