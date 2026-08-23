#include "display_mgr.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// True only after a successful displayInit(). Every other entry point bails out
// while this is false, so skipping displayInit() on an unwatched (headless)
// wake turns all display calls into safe no-ops — no I2C traffic, OLED stays
// powered down from the previous sleep.
static bool s_inited = false;

// Row geometry — 7 rows at textSize=1 (6×8 px per char, 1 px gap)
static const uint8_t LINE_H  =  9;
static const uint8_t ROW0_Y  =  0;
static const uint8_t ROW1_Y  =  LINE_H;
static const uint8_t ROW2_Y  =  LINE_H * 2;
static const uint8_t ROW3_Y  =  LINE_H * 3;
static const uint8_t ROW4_Y  =  LINE_H * 4;
static const uint8_t ROW5_Y  =  LINE_H * 5;
static const uint8_t ROW6_Y  =  LINE_H * 6;  // y=54; chars end at y=62 — fits 64 px

// ================================================================
//  displayInit — start the OLED on the Wire (I2C) bus and arm the panel. This
//  is the ONLY function that sets s_inited, so if it's skipped on a headless
//  wake every other display*() call becomes a safe no-op.
// ================================================================
void displayInit() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("[OLED] Init FAILED"));
    return;
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.display();
  s_inited = true;
  Serial.println(F("[OLED] Init OK"));
}

// ================================================================
//  displaySplash — show a two-line boot/status screen inside a border (line1 at
//  size 2 if it fits the width, else size 1). Used for "Booting…", "LoRa
//  init…", "WiFi…", "LOW BATTERY", etc.
// ================================================================
void displaySplash(const char* line1, const char* line2) {
  if (!s_inited) return;
  oled.clearDisplay();
  oled.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SSD1306_WHITE);

  // Use size-2 text for line1 only if it fits the panel width (12 px/char
  // at size 2, 6 px/char at size 1); otherwise drop to size 1 so long
  // strings like "SENSOR 1.1.0" or "LOW BATTERY" don't clip off the edge.
  bool fitsLarge = (strlen(line1) * 12u) <= (OLED_WIDTH - 8u);
  oled.setTextSize(fitsLarge ? 2 : 1);
  oled.setCursor(4, fitsLarge ? 8 : 12);
  oled.print(line1);

  oled.setTextSize(1);
  oled.setCursor(4, 42);
  oled.print(line2);
  oled.display();
}

// ================================================================
//  displayData — single static screen, no page cycling.
//
//  With deep sleep the device is only awake for a few seconds per
//  cycle.  All information is packed onto one screen so nothing is
//  missed.  Layout (128×64 px, textSize=1):
//
//   Row 0 (y= 0): 2025-06-01 12:00:00    ← timestamp
//   Row 1 (y= 9): R:200.0 K:199.8cm      ← raw + Kalman distance
//   Row 2 (y=18): WL: 100.2 cm           ← water level
//   Row 3 (y=27): T:28.5C  H:65.2%       ← temp + humidity
//   Row 4 (y=36): Bat:3.82V  #1234       ← battery + wake count
//   Row 5 (y=45): LoRa:ACK WiFi:OK       ← comms result
//   Row 6 (y=54): SD:OK  NextWifi:9      ← SD + wakes until WiFi
// ================================================================
void displayData(const SensorData& data,
                 bool              sdOk,
                 int8_t            loraStatus,
                 int8_t            wifiStatus,
                 uint32_t          wakeCount,
                 uint32_t          nextWifiIn) {
  if (!s_inited) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // Row 0 — timestamp (use first 19 chars: "YYYY-MM-DDTHH:MM:SS")
  oled.setCursor(0, ROW0_Y);
  {
    const char* ts = data.isoTimestamp;
    // Replace 'T' separator with ' ' for readability
    char buf[20];
    strncpy(buf, ts, 19);
    buf[19] = '\0';
    if (buf[10] == 'T') buf[10] = ' ';
    oled.print(buf);
  }

  // Row 1 — raw distance and Kalman-filtered distance
  oled.setCursor(0, ROW1_Y);
  if (data.distanceValid) {
    oled.printf("R:%.1f K:%.1fcm", data.distanceRaw, data.distanceCm);
  } else {
    oled.print(F("Dist: ---"));
  }

  // Row 2 — water level
  oled.setCursor(0, ROW2_Y);
  if (data.distanceValid) {
    oled.printf("WL: %.2f cm", data.waterLevelCm);
  } else {
    oled.print(F("WL: ---"));
  }

  // Row 3 — temperature and humidity
  oled.setCursor(0, ROW3_Y);
  if (data.envValid) {
    oled.printf("T:%.1fC  H:%.1f%%", data.tempC, data.humidity);
  } else {
    oled.print(F("T:--- H:---"));
  }

  // Row 4 — battery voltage + wake count
  oled.setCursor(0, ROW4_Y);
  if (data.batValid) {
    oled.printf("Bat:%.2fV  #%lu", data.batV, (unsigned long)wakeCount);
  } else {
    oled.printf("Bat:---  #%lu", (unsigned long)wakeCount);
  }

  // Row 5 — LoRa and WiFi status
  oled.setCursor(0, ROW5_Y);
  {
    const char* loraStr;
    switch (loraStatus) {
      case  1: loraStr = "ACK";  break;
      case  2: loraStr = "SENT"; break;
      case  0: loraStr = "FAIL"; break;
      default: loraStr = "SKIP"; break;
    }
    const char* wifiStr;
    switch (wifiStatus) {
      case  1: wifiStr = "OK";   break;
      case  0: wifiStr = "FAIL"; break;
      default: wifiStr = "SKIP"; break;
    }
    oled.printf("LoRa:%-4s WiFi:%-4s", loraStr, wifiStr);
  }

  // Row 6 — SD status and wakes until next WiFi upload
  oled.setCursor(0, ROW6_Y);
  oled.printf("SD:%-4s NextWifi:%-2lu",
              sdOk ? "OK" : "FAIL",
              (unsigned long)nextWifiIn);

  oled.display();
}

// ================================================================
//  OTA progress bar
// ================================================================
void displayOTAProgress(int percent) {
  if (!s_inited) return;
  if (percent < 0)   percent = 0;
  if (percent > 100) percent = 100;

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(16, 4);
  oled.print(F("OTA  UPDATE"));

  const int16_t BX = 2, BY = 18, BW = 124, BH = 12;
  oled.drawRect(BX, BY, BW, BH, SSD1306_WHITE);
  int16_t fill = (int16_t)((BW - 2) * percent / 100);
  if (fill > 0) oled.fillRect(BX + 1, BY + 1, fill, BH - 2, SSD1306_WHITE);

  String pctStr = String(percent) + "%";
  int16_t px = (OLED_WIDTH - (int16_t)(pctStr.length() * 6)) / 2;
  oled.setCursor(px, 36);
  oled.print(pctStr);

  oled.setCursor(10, 52);
  oled.print(F("DO NOT UNPLUG"));

  oled.display();
}

// ================================================================
//  displayOff — disable the OLED panel before deep sleep.
//  SSD1306_DISPLAYOFF turns off the charge pump, dropping the panel
//  from ~7–8 mA to microamps.  Without this the OLED would dominate
//  total current and negate the deep-sleep power savings.
//  Must be called before Wire.end() since the command goes over I2C.
// ================================================================
void displayOff() {
  if (!s_inited) return;
  oled.ssd1306_command(SSD1306_DISPLAYOFF);
}
