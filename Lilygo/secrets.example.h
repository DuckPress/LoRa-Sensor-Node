#pragma once
// ================================================================
//  DEPRECATED for the node — DO NOT USE.
//
//  The sensor node no longer compiles its credentials into the binary
//  (that would leak them in the OTA-published firmware.bin). Its WiFi
//  networks, GAS host/path and LoRa token are now read at boot from the
//  SD-card file /secrets.txt — see secrets.example.txt and secret_store.h.
//
//  This file is kept only so an old build referencing it still resolves;
//  it defines nothing. The GATEWAY (Xiao/) still uses its own compile-time
//  secrets.h — that binary is USB-flashed only and never published.
// ================================================================
