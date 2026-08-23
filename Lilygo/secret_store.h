#pragma once
#include <Arduino.h>
#include <stdint.h>

// ================================================================
//  secret_store — runtime credentials for the NODE, kept OUT of the binary.
//
//  The node is the only device whose firmware.bin gets published (OTA over a
//  PUBLIC GitHub repo). Compile-time secrets (a #included secrets.h) would be
//  baked into that binary as plaintext strings — trivially recovered with
//  `strings firmware.bin`. So the node reads its WiFi networks, GAS host/path
//  and LoRa token from the SD-card file /secrets.txt instead, caching them in
//  NVS/flash so they survive the card being removed or a transient read glitch.
//  The compiled binary therefore contains NO credentials and is safe to publish.
//
//  Provision by placing /secrets.txt on the SD card (see secrets.example.txt):
//      wifi=MySSID,MyPassword
//      wifi=SecondSSID,SecondPassword
//      gas_host=script.google.com
//      gas_path=/macros/s/AKfy.../exec
//      lora_token=            # blank = LoRa auth disabled (must match gateway)
//
//  The gateway (Xiao) is USB-flashed only — its binary is never published — so
//  it keeps its compile-time secrets.h unchanged.
// ================================================================

constexpr uint8_t SECRET_MAX_WIFI = 4;

// Load secrets: try the SD file first (and refresh the NVS cache from it), then
// fall back to the NVS cache if the file is absent/unreadable. Call ONCE after
// sdInit(), before any WiFi/LoRa/GAS use. Returns true if usable secrets were
// loaded from either source.
bool secretsLoad();

// True once secretsLoad() has found usable secrets (>=1 WiFi network + a GAS
// host/path). WiFi-less operation (LoRa only) is still allowed if only the
// GAS/WiFi entries are missing — accessors just return empties.
bool secretsAvailable();

uint8_t     secretWifiCount();
const char* secretWifiSsid(uint8_t i);   // "" if i out of range
const char* secretWifiPass(uint8_t i);   // "" if i out of range
const char* secretGasHost();             // "" if unset
const char* secretGasPath();             // "" if unset
const char* secretLoraToken();           // "" = LoRa auth disabled
