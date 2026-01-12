#include "rtc_time.h"
#include "config.h"
#include <Wire.h>
#include <Arduino.h>

// DS3231 I2C address
#define DS3231_ADDR 0x68

// DS3231 register addresses
#define DS3231_SECONDS_REG 0x00
#define DS3231_CONTROL_REG 0x0E

static bool rtcInitialized = false;

// BCD to decimal conversion
static uint8_t bcdToDec(uint8_t val) {
  return (val >> 4) * 10 + (val & 0x0F);
}

// Decimal to BCD conversion
static uint8_t decToBcd(uint8_t val) {
  return ((val / 10) << 4) | (val % 10);
}

void initRTC() {
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  Wire.setClock(400000);

  // Check if DS3231 is responding
  Wire.beginTransmission(DS3231_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[RTC] ERROR: DS3231 not found at 0x68");
    rtcInitialized = false;
    return;
  }

  // Enable oscillator (clear EOSC bit in control register)
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(DS3231_CONTROL_REG);
  Wire.endTransmission(false);
  Wire.requestFrom(DS3231_ADDR, 1);
  uint8_t control = Wire.read();
  
  control &= ~(1 << 7);  // Clear EOSC bit to enable oscillator
  
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(DS3231_CONTROL_REG);
  Wire.write(control);
  Wire.endTransmission();

  rtcInitialized = true;
  
  struct tm timeinfo;
  if (getRTCTime(&timeinfo)) {
    Serial.printf("[RTC] Initialized: %04d-%02d-%02d %02d:%02d:%02d\n",
      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }
}

bool getRTCTime(struct tm* timeinfo) {
  if (!rtcInitialized) return false;

  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(DS3231_SECONDS_REG);
  Wire.endTransmission(false);
  Wire.requestFrom(DS3231_ADDR, 7);

  if (Wire.available() < 7) {
    return false;
  }

  uint8_t seconds = bcdToDec(Wire.read() & 0x7F);
  uint8_t minutes = bcdToDec(Wire.read());
  uint8_t hours = bcdToDec(Wire.read() & 0x3F);
  Wire.read();  // Skip day of week
  uint8_t date = bcdToDec(Wire.read());
  uint8_t month = bcdToDec(Wire.read() & 0x1F);
  uint8_t year = bcdToDec(Wire.read());

  timeinfo->tm_sec = seconds;
  timeinfo->tm_min = minutes;
  timeinfo->tm_hour = hours;
  timeinfo->tm_mday = date;
  timeinfo->tm_mon = month - 1;  // 0-11
  timeinfo->tm_year = year + 100;  // Years since 1900, DS3231 stores 2-digit year
  timeinfo->tm_isdst = 0;

  return true;
}

time_t getRTCTimestamp() {
  struct tm timeinfo;
  if (!getRTCTime(&timeinfo)) {
    return 0;
  }
  return mktime(&timeinfo);
}

void getTimeString(char* buffer, size_t buflen) {
  if (!rtcInitialized || buflen < 13) return;

  struct tm timeinfo;
  if (!getRTCTime(&timeinfo)) {
    snprintf(buffer, buflen, "TIME_ERROR");
    return;
  }

  uint32_t currentMicros = micros();
  uint32_t millisPart = (currentMicros % 1000000) / 1000;

  snprintf(buffer, buflen, "%02d:%02d:%02d.%03lu",
    timeinfo.tm_hour,
    timeinfo.tm_min,
    timeinfo.tm_sec,
    millisPart);
}
