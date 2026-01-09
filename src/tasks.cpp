#include "tasks.h"
#include "config.h"
#include "imu_sensor.h"
#include "feature_processing.h"
#include "ble_service.h"
#include <Arduino.h>

// Global data packets
ImuPacket imuPacket;
FeaturePacket featurePacket;

// Mutexes for thread-safe access
SemaphoreHandle_t imuDataMutex;
SemaphoreHandle_t featureDataMutex;

/* ---------- IMU Sampling Task (50 Hz) ---------- */
void imuSamplingTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS);

  while (true) {
    int16_t rawAccel[3], rawGyro[3], rawTemp;
    
    // Read sensor data
    if (readIMUData(rawAccel, rawGyro, rawTemp)) {
      // Lock mutex and store raw IMU data
      if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5))) {
        imuPacket.timestamp = millis();
        imuPacket.ax = rawAccel[0];
        imuPacket.ay = rawAccel[1];
        imuPacket.az = rawAccel[2];
        imuPacket.gx = rawGyro[0];
        imuPacket.gy = rawGyro[1];
        imuPacket.gz = rawGyro[2];
        imuPacket.temp = rawTemp;
        xSemaphoreGive(imuDataMutex);
      }
    }

    // Wait for next sample period (precise timing)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* ---------- Feature Computation Task (50 Hz) ---------- */
void featureComputationTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(FEATURE_UPDATE_PERIOD_MS);
  uint16_t updateCount = 0;

  while (true) {
    ImuPacket localImuData;
    
    // Copy current IMU data
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(10))) {
      localImuData = imuPacket;
      xSemaphoreGive(imuDataMutex);
      
      // Compute features
      updateRMS(localImuData.ax, localImuData.ay, localImuData.az);
      detectStep(localImuData.ax, localImuData.ay, localImuData.az);
      updateOrientation(localImuData.ax, localImuData.ay, localImuData.az,
                       localImuData.gx, localImuData.gy, localImuData.gz);
      
      // Build feature packet
      if (xSemaphoreTake(featureDataMutex, pdMS_TO_TICKS(10))) {
        featurePacket.timestamp = localImuData.timestamp;
        featurePacket.ax = localImuData.ax;
        featurePacket.ay = localImuData.ay;
        featurePacket.az = localImuData.az;
        featurePacket.gx = localImuData.gx;
        featurePacket.gy = localImuData.gy;
        featurePacket.gz = localImuData.gz;
        featurePacket.rms = (int16_t)(getRMSValue() * 100.0f);      // Scale by 100
        featurePacket.pitch = (int16_t)(getPitch() * 100.0f);       // Scale by 100
        featurePacket.roll = (int16_t)(getRoll() * 100.0f);         // Scale by 100
        featurePacket.stepCount = getStepCount();
        xSemaphoreGive(featureDataMutex);
      }
      
      // Print status every 25 updates (every 0.5 seconds at 50 Hz)
      if (++updateCount % 25 == 0) {
        Serial.printf("T:%u A:%d,%d,%d G:%d,%d,%d | Pitch:%.1f° Roll:%.1f° Steps:%u RMS:%.1f\n",
          localImuData.timestamp,
          localImuData.ax, localImuData.ay, localImuData.az,
          localImuData.gx, localImuData.gy, localImuData.gz,
          getPitch(), getRoll(), getStepCount(), getRMSValue());
      }
    }

    // Wait for next update period
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* ---------- BLE Task (10 Hz) ---------- */
void bleTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(BLE_NOTIFY_PERIOD_MS);

  while (true) {
    if (isBLEConnected()) {
      FeaturePacket packet;
      
      // Copy feature packet atomically
      if (xSemaphoreTake(featureDataMutex, pdMS_TO_TICKS(10))) {
        packet = featurePacket;
        xSemaphoreGive(featureDataMutex);

        // Send feature packet via BLE
        NimBLECharacteristic* pChar = getBLECharacteristic();
        if (pChar) {
          pChar->setValue((uint8_t*)&packet, sizeof(packet));
          pChar->notify();
        }
      }
    }

    // Wait for next notification period
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* ---------- Initialize All Tasks ---------- */
void initTasks() {
  // Create mutexes for data sharing
  imuDataMutex = xSemaphoreCreateMutex();
  featureDataMutex = xSemaphoreCreateMutex();

  // Create FreeRTOS tasks
  // IMU sampling task (highest priority, core 1)
  xTaskCreatePinnedToCore(imuSamplingTask, "IMU_Task", 4096, NULL, 3, NULL, 1);
  
  // Feature computation task (medium priority, core 1)
  xTaskCreatePinnedToCore(featureComputationTask, "Feature_Task", 8192, NULL, 2, NULL, 1);
  
  // BLE notification task (lower priority, core 0 for BLE stack)
  xTaskCreatePinnedToCore(bleTask, "BLE_Task", 4096, NULL, 1, NULL, 0);

  Serial.print("IMU sampling at ");
  Serial.print(IMU_SAMPLE_RATE_HZ);
  Serial.println(" Hz");
  Serial.print("Feature updates at ");
  Serial.print(FEATURE_UPDATE_RATE_HZ);
  Serial.println(" Hz");
  Serial.print("BLE notifications at ");
  Serial.print(BLE_NOTIFY_RATE_HZ);
  Serial.println(" Hz");
}
