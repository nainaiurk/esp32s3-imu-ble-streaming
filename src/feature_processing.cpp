#include "feature_processing.h"
#include "config.h"
#include <Arduino.h>

// RMS variables
static int32_t accSqSum = 0;
static uint16_t rmsCount = 0;
static float rmsValue = 0;

// Step detection variables
static float gravityX = 0, gravityY = 0, gravityZ = 0;
static int32_t prevDynMagSq = 0;
static int32_t lastDynamicMagSq = 0;
static uint32_t stepCount = 0;
static uint32_t lastStepTime = 0;
static const int32_t STEP_THRESHOLD_SQ = (int32_t)(STEP_THRESHOLD * STEP_THRESHOLD);

// Orientation variables
static float pitch = 0.0f;
static float roll = 0.0f;
static const float DT = 1.0f / FEATURE_UPDATE_RATE_HZ;

void initFeatureProcessing() {
  accSqSum = 0;
  rmsCount = 0;
  rmsValue = 0;
  gravityX = gravityY = gravityZ = 0;
  prevDynMagSq = lastDynamicMagSq = 0;
  stepCount = 0;
  lastStepTime = 0;
  pitch = roll = 0.0f;
}

void updateRMS(int16_t ax, int16_t ay, int16_t az) {
  int32_t magSq = (int32_t)ax*ax + (int32_t)ay*ay + (int32_t)az*az;
  accSqSum += magSq;
  rmsCount++;

  if (rmsCount >= RMS_WINDOW) {
    rmsValue = sqrt((float)accSqSum / RMS_WINDOW);
    accSqSum = 0;
    rmsCount = 0;
  }
}
 
float getRMSValue() {
  return rmsValue;
}

void detectStep(int16_t ax, int16_t ay, int16_t az) {
  // Low-pass filter to estimate gravity (slowly adapts to orientation)
  gravityX = GRAVITY_ALPHA * gravityX + (1.0f - GRAVITY_ALPHA) * ax;
  gravityY = GRAVITY_ALPHA * gravityY + (1.0f - GRAVITY_ALPHA) * ay;
  gravityZ = GRAVITY_ALPHA * gravityZ + (1.0f - GRAVITY_ALPHA) * az;
  
  // Remove gravity to get dynamic (linear) acceleration
  float dynX = ax - gravityX;
  float dynY = ay - gravityY;
  float dynZ = az - gravityZ;
  
  // Calculate magnitude squared of dynamic acceleration
  int32_t dynMagSq = (int32_t)(dynX*dynX + dynY*dynY + dynZ*dynZ);
  
  uint32_t now = millis();
  
  // Three-point peak detection: prev < last > current (local maximum)
  if (prevDynMagSq < lastDynamicMagSq &&
      lastDynamicMagSq > dynMagSq &&
      lastDynamicMagSq > STEP_THRESHOLD_SQ &&
      (now - lastStepTime) >= MIN_STEP_INTERVAL_MS) {
    stepCount++;
    lastStepTime = now;
  }
  
  prevDynMagSq = lastDynamicMagSq;
  lastDynamicMagSq = dynMagSq;
}

uint32_t getStepCount() {
  return stepCount;
}

void updateOrientation(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
  // Convert accelerometer to g's
  float ax_g = ax / ACCEL_SCALE_4G;
  float ay_g = ay / ACCEL_SCALE_4G;
  float az_g = az / ACCEL_SCALE_4G;
  
  // Calculate pitch and roll from accelerometer (in degrees)
  float accelPitch = atan2(ay_g, sqrt(ax_g*ax_g + az_g*az_g)) * 180.0f / PI;
  float accelRoll = atan2(-ax_g, az_g) * 180.0f / PI;
  
  // Convert gyroscope to °/s and integrate
  float gx_dps = gx / GYRO_SCALE_500DPS;
  float gy_dps = gy / GYRO_SCALE_500DPS;
  
  // Integrate gyro rates to get angles (gyro measures rotation rate)
  float gyroPitch = pitch + gx_dps * DT;
  float gyroRoll = roll - gy_dps * DT;
  
  // Complementary filter: 98% gyro (short-term accuracy), 2% accel (long-term stability)
  pitch = COMPLEMENTARY_ALPHA * gyroPitch + (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;
  roll = COMPLEMENTARY_ALPHA * gyroRoll + (1.0f - COMPLEMENTARY_ALPHA) * accelRoll;
}

float getPitch() {
  return pitch;
}

float getRoll() {
  return roll;
}
