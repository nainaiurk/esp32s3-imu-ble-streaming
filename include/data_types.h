#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <stdint.h>

struct __attribute__((packed)) ImuPacket {
  uint32_t timestamp;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;
};


struct __attribute__((packed)) FeaturePacket {
  uint32_t timestamp;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t rms;    // RMS * 100 (scaled)
  int16_t pitch;  // degrees * 100 (scaled)
  int16_t roll;   // degrees * 100 (scaled)
  uint32_t stepCount;
};

#endif
