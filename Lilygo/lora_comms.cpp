#include "lora_comms.h"
#include "config.h"
#include "secret_store.h"   // LoRa auth token now comes from the SD card, not the binary

#include <RadioLib.h>
#include <SPI.h>
#include <esp_task_wdt.h>

// Dedicated HSPI bus for the LoRa radio — separate from SD (default SPI/FSPI)
static SPIClass loraSPI(HSPI);

static SX1262 radio = new Module(
  PIN_LORA_NSS,
  PIN_LORA_DIO1,
  PIN_LORA_RST,
  PIN_LORA_BUSY,
  loraSPI
);

static bool     s_loraReady   = false;
static int16_t  s_lastAckRSSI = 0;
static float    s_lastAckSNR  = 0.0f;
// Gateway's UTC+8 epoch parsed from the last ACK ("ACK:<seq>:<epoch>"), or 0 if
// the ACK carried no time (older gateway, or its clock not yet NTP-valid). Lets
// the node correct its RTC over LoRa without any WiFi/NTP of its own.
static uint32_t s_ackGwEpoch  = 0;

// Sequence number persists across deep sleep so the gateway can detect
// missed packets even when the node wakes and re-initialises each cycle.
RTC_DATA_ATTR static uint16_t s_seqNum = 0;

// ================================================================
//  loraInit — bring up the SX1262 on its dedicated HSPI bus and set the AS923
//  RF parameters that MUST match the gateway exactly. Also enables the T3-S3's
//  1.8 V TCXO and DIO2-driven RF switch. Returns false on any radio error
//  (the caller then skips TX for this wake).
// ================================================================
bool loraInit() {
  Serial.println(F("[LoRa] Initialising SX1262 (AS923-1, 923.2 MHz)..."));

  loraSPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);

  pinMode(PIN_LORA_NSS, OUTPUT);
  digitalWrite(PIN_LORA_NSS, HIGH);
  delay(10);

  pinMode(PIN_LORA_BUSY, INPUT);
  Serial.printf("[LoRa] BUSY pin (GPIO%d) idle: %s\n",
                PIN_LORA_BUSY,
                digitalRead(PIN_LORA_BUSY) ? "HIGH (unexpected)" : "LOW (ok)");

  int state = radio.begin(
    LORA_FREQUENCY,
    LORA_BW,
    LORA_SF,
    LORA_CR,
    RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
    LORA_TX_POWER,
    8   // preamble symbols
  );

  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] Init FAILED, error: %d\n", state);
    s_loraReady = false;
    return false;
  }

  // T3-S3 uses a 1.8 V TCXO (not a crystal)
  radio.setTCXO(1.8f);

  // DIO2 drives the onboard RF switch
  radio.setDio2AsRfSwitch(true);

  radio.setCurrentLimit(140.0f);

  s_loraReady = true;
  Serial.printf("[LoRa] Init OK — %.1f MHz  SF%d  BW%.0f  CR4/%d  %d dBm\n",
                LORA_FREQUENCY, LORA_SF, LORA_BW, LORA_CR, LORA_TX_POWER);
  return true;
}

// ================================================================
//  SX126x RX timeout units: 1 tick = 15.625 µs (64 ticks per ms)
// ================================================================
static uint32_t msToRadioTicks(uint32_t ms) { return ms * 64UL; }

// ================================================================
//  JSON payload builder
//
//  Fields:
//    ts  — ISO-8601 timestamp from DS3231
//    d   — Kalman-filtered distance (cm)
//    dr  — trimmed-mean distance before Kalman (cm) — raw reading
//    wl  — water level = SENSOR_HEIGHT_CM - d
//    t   — temperature (°C)
//    h   — relative humidity (%)
//    rv  — RTC valid flag (0/1)
//    ev  — env sensor valid flag (0/1)
//    rssi— WiFi RSSI at time of tx (or 0 if no WiFi)
//    seq — monotonic sequence number (persists across deep sleep)
//    id  — node identity (NODE_ID) for multi-node gateways
//    bat — battery voltage (V), -1 if invalid
//    sd  — burst standard deviation (cm), -1 if unknown  [QC]
//    n   — burst sample count                            [QC]
//    sm  — survey-mode flag (0/1)                        [QC]
//    mv  — mount-moved flag from optional tilt sensor (0/1) [QC]
// ================================================================
static size_t buildPayload(char* buf, size_t bufSize,
                           const SensorData& data,
                           int32_t wifiRssi, uint16_t seq) {
  int n = snprintf(buf, bufSize,
    "{"
      "\"ts\":\"%s\","
      "\"d\":%.2f,"
      "\"dr\":%.2f,"
      "\"wl\":%.2f,"
      "\"t\":%.2f,"
      "\"h\":%.1f,"
      "\"rv\":%d,"
      "\"ev\":%d,"
      "\"rssi\":%ld,"
      "\"seq\":%u,"
      "\"id\":%u,"
      "\"bat\":%.2f,"
      "\"sd\":%.2f,"
      "\"n\":%u,"
      "\"sm\":%d,"
      "\"mv\":%d",         // NOTE: object intentionally left open — see below
    data.isoTimestamp,
    data.distanceValid ? data.distanceCm   : -1.0f,
    data.distanceValid ? data.distanceRaw  : -1.0f,
    data.distanceValid ? data.waterLevelCm : -1.0f,
    data.envValid      ? data.tempC        : -999.0f,
    data.envValid      ? data.humidity     : -1.0f,
    data.rtcValid      ? 1 : 0,
    data.envValid      ? 1 : 0,
    (long)wifiRssi,
    (unsigned)seq,
    (unsigned)NODE_ID,
    data.batValid      ? data.batV         : -1.0f,
    data.burstSd,
    (unsigned)data.burstN,
    data.surveyMode    ? 1 : 0,
    data.moved         ? 1 : 0);

  if (n <= 0 || (size_t)n >= bufSize) {
    Serial.println(F("[LoRa] Payload truncated!"));
    return 0;
  }

  // Append the shared-secret auth token ("k") only when one is configured, so
  // an empty token keeps the payload byte-identical to older firmware. The
  // gateway requires this to match before it ACKs/uploads the packet.
  const char* loraToken = secretLoraToken();
  if (loraToken[0] != '\0') {
    int m = snprintf(buf + n, bufSize - n, ",\"k\":\"%s\"", loraToken);
    if (m <= 0 || (size_t)(n + m) >= bufSize) {
      Serial.println(F("[LoRa] Payload truncated (token)!"));
      return 0;
    }
    n += m;
  }

  // Close the JSON object.
  int c = snprintf(buf + n, bufSize - n, "}");
  if (c <= 0 || (size_t)(n + c) >= bufSize) {
    Serial.println(F("[LoRa] Payload truncated (brace)!"));
    return 0;
  }
  return (size_t)(n + c);
}

// Redacts the "k":"<token>" field before logging a payload — the shared LoRa
// auth token is the link's only authentication and must never land in a
// serial capture or bug-report log that could reach someone without access
// to the SD-card /secrets.txt.
static void redactTokenForLog(const char* payload, char* out, size_t outSize) {
  strncpy(out, payload, outSize - 1);
  out[outSize - 1] = '\0';
  char* k = strstr(out, "\"k\":\"");
  if (k) {
    char* v = k + 5;   // first char of the token value
    while (*v && *v != '"') *v++ = '*';
  }
}

// ================================================================
//  loraSend — transmit one reading and wait for the gateway's ACK.
//
//  Builds the JSON payload, transmits it, then listens up to
//  LORA_ACK_TIMEOUT_MS for an "ACK:<seq>" whose seq matches what we just sent
//  (this rejects a stale ACK from an abandoned earlier packet). Retries up to
//  LORA_MAX_RETRIES. The sequence number persists across deep sleep so the
//  gateway can spot gaps/duplicates. Returns OK / NO_ACK / TX_ERROR / NOT_INIT.
// ================================================================
LoRaSendResult loraSend(const SensorData& data, int32_t wifiRssi) {
  if (!s_loraReady) {
    Serial.println(F("[LoRa] Not initialised"));
    return LoRaSendResult::NOT_INIT;
  }

  char   payload[LORA_PAYLOAD_SIZE];
  size_t payloadLen = buildPayload(payload, sizeof(payload),
                                   data, wifiRssi, s_seqNum);
  if (payloadLen == 0) return LoRaSendResult::TX_ERROR;

  char logPayload[LORA_PAYLOAD_SIZE];
  redactTokenForLog(payload, logPayload, sizeof(logPayload));
  Serial.printf("[LoRa] TX seq=%u  len=%u\n       %s\n",
                s_seqNum, (unsigned)payloadLen, logPayload);

  // Tracks whether the radio ever accepted a packet for transmission.
  // Lets us distinguish a true radio/TX failure (TX_ERROR) from a
  // successful transmit that simply never got an ACK (NO_ACK).
  bool anyTxOk = false;

  // Each attempt = transmit once, then wait for a matching ACK. Repeat up to
  // LORA_MAX_RETRIES times; the first good ACK returns OK and exits immediately.
  for (uint8_t attempt = 1; attempt <= LORA_MAX_RETRIES; attempt++) {
    // --- Transmit ---
    int txState = radio.transmit((uint8_t*)payload, payloadLen);

    if (txState != RADIOLIB_ERR_NONE) {
      Serial.printf("[LoRa] TX error %d (attempt %d/%d)\n",
                    txState, attempt, LORA_MAX_RETRIES);
      delay(LORA_RETRY_DELAY_MS);
      continue;
    }

    anyTxOk = true;
    // getTimeOnAir() returns microseconds (uint32_t) — convert to ms. The old
    // "%.0f" with a uint32_t arg was undefined behaviour and printed 0.
    Serial.printf("[LoRa] TX done (attempt %d)  airtime=%lu ms\n",
                  attempt, (unsigned long)(radio.getTimeOnAir(payloadLen) / 1000UL));

    // --- Listen for ACK ---
    // Use startReceive() with the hardware RX timeout, then poll IRQ
    // flags. This avoids the RadioLib blocking receive() whose second
    // argument is a packet-length hint, NOT a millisecond timeout.
    radio.startReceive(msToRadioTicks(LORA_ACK_TIMEOUT_MS));

    uint32_t waitStart = millis();
    const uint32_t kBudget = LORA_ACK_TIMEOUT_MS + 200;  // software guard slightly > the HW window

    // The gateway echoes the seq it received ("ACK:<seq>"). Requiring the
    // echo to match the seq we just sent rejects a stale ACK left over
    // from a previous (already-abandoned) transmission.
    char expectedAck[16];
    snprintf(expectedAck, sizeof(expectedAck), "ACK:%u", (unsigned)s_seqNum);
    const size_t expLen = strlen(expectedAck);
    s_ackGwEpoch = 0;   // reset — populated below only if this ACK carries a time

    bool gotAck = false;

    // Poll the radio's IRQ flags instead of a blocking receive, so we can keep
    // feeding the watchdog while waiting up to ~3 s for the gateway's ACK.
    while (millis() - waitStart < kBudget) {
      esp_task_wdt_reset();
      uint16_t irq = radio.getIrqFlags();

      if (irq & RADIOLIB_SX126X_IRQ_RX_DONE) {   // a packet arrived → read it out
        String ackStr;
        int rxState = radio.readData(ackStr);
        if (rxState == RADIOLIB_ERR_NONE) {
          ackStr.trim();
          // Accept "ACK:<seq>" (older gateway) OR "ACK:<seq>:<epoch>" (newer:
          // the gateway appends its UTC+8 epoch so we can correct the RTC over
          // LoRa). Require the char right after our seq to be ':' so a short
          // "ACK:12" can't prefix-match a stale "ACK:123".
          bool match = (ackStr == expectedAck);
          if (!match && ackStr.length() > expLen &&
              ackStr.startsWith(expectedAck) && ackStr.charAt(expLen) == ':') {
            match = true;
            s_ackGwEpoch = (uint32_t)strtoul(ackStr.c_str() + expLen + 1, nullptr, 10);
          }
          if (match) { gotAck = true; break; }
          Serial.printf("[LoRa] Unexpected/stale response: \"%s\" (wanted \"%s\")\n",
                        ackStr.c_str(), expectedAck);
        } else {
          Serial.printf("[LoRa] RX error %d while waiting for ACK\n", rxState);
        }
        // Not our ACK (foreign packet / noise / stale echo) — keep listening
        // for the REMAINDER of the window instead of burning a whole retry.
        uint32_t elapsed = millis() - waitStart;
        if (elapsed >= LORA_ACK_TIMEOUT_MS) break;
        radio.startReceive(msToRadioTicks(LORA_ACK_TIMEOUT_MS - elapsed));
        continue;
      }
      if (irq & RADIOLIB_SX126X_IRQ_TIMEOUT) break;  // HW RX window expired — no ACK
      delay(5);                                  // brief idle between IRQ polls
    }

    if (gotAck) {
      s_lastAckRSSI = radio.getRSSI();
      s_lastAckSNR  = radio.getSNR();
      s_seqNum++;
      Serial.printf("[LoRa] ACK seq=%u  RSSI=%d dBm  SNR=%.1f dB\n",
                    (unsigned)(s_seqNum - 1), s_lastAckRSSI, s_lastAckSNR);
      return LoRaSendResult::OK;
    }
    Serial.printf("[LoRa] No ACK within window (attempt %d/%d)\n",
                  attempt, LORA_MAX_RETRIES);

    if (attempt < LORA_MAX_RETRIES) {
      Serial.printf("[LoRa] Retrying in %d ms...\n", LORA_RETRY_DELAY_MS);
      delay(LORA_RETRY_DELAY_MS);
    }
  }

  s_seqNum++;
  if (anyTxOk) {
    Serial.println(F("[LoRa] All retries exhausted — no ACK"));
    return LoRaSendResult::NO_ACK;
  }
  Serial.println(F("[LoRa] All transmit attempts failed — radio error"));
  return LoRaSendResult::TX_ERROR;
}

void loraShutdown() {
  if (!s_loraReady) return;
  // SX1262 sleep draws ~0.5 µA vs ~1.5 mA in standby.
  // The radio is re-initialised from scratch on the next wake via loraInit().
  radio.sleep();
  s_loraReady = false;
  Serial.println(F("[LoRa] Radio sleeping"));
}

// Small accessors for state cached during the last send (used by logging / the
// gateway has its own copies for the heartbeat).
bool    loraReady()       { return s_loraReady; }   // radio initialised this wake?
int16_t  loraLastAckRSSI()     { return s_lastAckRSSI; } // RSSI of last ACK (dBm)
float    loraLastAckSNR()      { return s_lastAckSNR; }  // SNR  of last ACK (dB)
uint32_t loraLastGatewayEpoch(){ return s_ackGwEpoch; }  // UTC+8 epoch from last ACK, 0 if none
