#pragma once
#include <WiFiClientSecure.h>

// ================================================================
//  net_tls.h — one place to configure TLS trust for every HTTPS
//  client on the node (telemetry uploads, pending flush, OTA).
//
//  When TLS_VERIFY is 1 (default) the server certificate is validated
//  against the ESP32 Arduino core's built-in Mozilla root CA bundle
//  (~130 roots). That bundle already contains the roots for BOTH
//  script.google.com (Google Trust Services) and github.com / its
//  release-download CDN, so uploads AND OTA are authenticated with no
//  certificate to hand-maintain or rotate.
//
//  This closes the previous setInsecure() hole — most importantly on
//  the OTA path, where an unauthenticated TLS session let a network
//  attacker serve malicious firmware (remote code execution).
//
//  If a build ever fails to link `rootca_crt_bundle_start`, or you
//  must talk to a host outside the standard root store, set TLS_VERIFY
//  to 0 here to fall back to the old (unauthenticated) behaviour.
// ================================================================
#ifndef TLS_VERIFY
#define TLS_VERIFY 1
#endif

#if TLS_VERIFY
// Provided by the ESP32 Arduino core; the linker fills these in from the
// precompiled x509 certificate bundle. The esp32 3.x setCACertBundle() needs
// the byte length, so we keep both the start and end symbols.
extern const uint8_t rootca_crt_bundle_start[] asm("_binary_x509_crt_bundle_start");
extern const uint8_t rootca_crt_bundle_end[]   asm("_binary_x509_crt_bundle_end");
#endif

// Apply the project's TLS trust policy to a freshly-created client.
static inline void applyTls(WiFiClientSecure& client) {
#if TLS_VERIFY
  client.setCACertBundle(rootca_crt_bundle_start,
                         (size_t)(rootca_crt_bundle_end - rootca_crt_bundle_start));
#else
  client.setInsecure();   // no certificate validation (legacy behaviour)
#endif
}
