#ifndef RTC_TIME_H
#define RTC_TIME_H

#include <stdint.h>
#include <time.h>

// Initialize RTC via I2C
void initRTC();

// Get current time as struct tm (populated from RTC)
bool getRTCTime(struct tm* timeinfo);

// Get current Unix timestamp (seconds since 1970)
time_t getRTCTimestamp();

// Get just time portion: "14:30:45.123"
// buffer must be at least 13 bytes
void getTimeString(char* buffer, size_t buflen);

#endif
