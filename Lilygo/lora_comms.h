#pragma once
#include "sensors.h"
#include <Arduino.h>

// Outcome of one loraSend() call. Lilygo.ino uses this to decide whether the
// reading still needs the WiFi backup upload: only OK means "the gateway has it".
enum class LoRaSendResult : uint8_t {
  OK       = 0,   // transmitted AND a matching seq-echoed ACK came back
  NO_ACK   = 1,   // transmitted OK but no matching ACK after all retries
  TX_ERROR = 2,   // radio never accepted the packet for transmission
  NOT_INIT = 3,   // loraInit() did not succeed this wake
};

// Initialise the SX1262 on its dedicated HSPI bus (1.8 V TCXO, DIO2 RF switch,
// AS923 params). Call once per wake before sending. Returns true on success.
bool           loraInit();

// Build the JSON payload from `data`, transmit it, then wait (with retries) for
// the gateway's "ACK:<seq>" echo. Returns the outcome enum above.
LoRaSendResult loraSend(const SensorData& data, int32_t wifiRssi = 0);

// Put the SX1262 into sleep mode before deep sleep (~0.5 µA vs ~1.5 mA standby).
void           loraShutdown();

bool           loraReady();        // true once loraInit() has succeeded this wake
int16_t        loraLastAckRSSI();  // RSSI (dBm) of the most recently received ACK
float          loraLastAckSNR();   // SNR  (dB)  of the most recently received ACK
// Gateway's UTC+8 epoch parsed from the last ACK ("ACK:<seq>:<epoch>"), or 0 if
// the ACK carried no time. Lets the node correct its RTC over LoRa (no WiFi).
uint32_t       loraLastGatewayEpoch();
