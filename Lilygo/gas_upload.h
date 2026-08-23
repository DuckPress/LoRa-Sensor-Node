#pragma once
#include <Arduino.h>

// Send one query string (everything that goes after "?") to the Google Apps
// Script web app over HTTPS GET, following the GAS 302 redirect. Returns true
// only when the response is HTTP 200 AND its JSON body reports "ok":true.
// Centralises the URL build, TLS policy (net_tls.h), redirect handling and
// timeout so the direct-upload and pending-flush paths can't drift.
//
// Caller must already hold a WiFi connection.
bool gasUpload(const char* query);

// Same transport, but also hands back the response body — used to READ from
// the backend (e.g. the remote node config / survey-mode flag). Returns true
// on HTTP 200 with an "ok":true body; bodyOut holds the body either way.
bool gasFetch(const char* query, String& bodyOut);

// Upload newline-delimited query-string rows through the Apps Script batch
// endpoint. `expectedRows` must match the number of lines. Returns true only
// when GAS explicitly accounted for every row; `badRows` are permanently
// invalid rows GAS rejected, not a transport failure.
bool gasUploadBatch(const String& rows, uint16_t expectedRows, uint16_t& badRows);
