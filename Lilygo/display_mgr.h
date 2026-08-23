#pragma once
#include "sensors.h"

void displayInit();
void displaySplash(const char* line1, const char* line2);

// loraStatus : -1=not tried, 0=TX fail, 1=ACKed, 2=sent/no-ACK
// wifiStatus : -1=not an upload wake / no data, 0=upload failed, 1=uploaded
// wakeCount  : total wakes since power-on (RTC_DATA_ATTR)
// nextWifiIn : wakes until the next WiFi-upload cycle
void displayData(const SensorData& data,
                 bool              sdOk,
                 int8_t            loraStatus,
                 int8_t            wifiStatus,
                 uint32_t          wakeCount,
                 uint32_t          nextWifiIn);

void displayOTAProgress(int percent);

// Switch the OLED panel off (charge pump disabled) to cut its ~7–8 mA draw
// to microamps during deep sleep. Must be called BEFORE Wire.end().
void displayOff();
