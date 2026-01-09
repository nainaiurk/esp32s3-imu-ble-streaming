#ifndef FEATURE_PROCESSING_H
#define FEATURE_PROCESSING_H

#include <stdint.h>
void initFeatureProcessing();
void updateRMS(int16_t ax, int16_t ay, int16_t az);
float getRMSValue();
void detectStep(int16_t ax, int16_t ay, int16_t az);
uint32_t getStepCount();
void updateOrientation(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
float getPitch();
float getRoll();

#endif
