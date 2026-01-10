#include "tasks.h"
#include "config.h"
#include "imu_sensor.h"
#include "feature_processing.h"
#include "ble_service.h"
#include "sd_card_data.h"
#include <Arduino.h>

// Global data packets
ImuPacket imuPacket;
FeaturePacket featurePacket;

// Mutexes for thread-safe access
SemaphoreHandle_t imuDataMutex;
SemaphoreHandle_t featureDataMutex;

// Event group for inter-task synchronization
EventGroupHandle_t taskEventGroup;

// ---------- IMU Sampling Task (50 Hz) ----------
void imuSamplingTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS);

  while (true) {
    int16_t rawAccel[3], rawGyro[3], rawTemp;
    
    if (readIMUData(rawAccel, rawGyro, rawTemp)) {
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

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ---------- Feature Computation Task (50 Hz) ----------
void featureComputationTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(FEATURE_UPDATE_PERIOD_MS);
  uint16_t updateCount = 0;

  while (true) {
    ImuPacket localImuData;
    
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(10))) {
      localImuData = imuPacket;
      xSemaphoreGive(imuDataMutex);
      
      updateRMS(localImuData.ax, localImuData.ay, localImuData.az);
      detectStep(localImuData.ax, localImuData.ay, localImuData.az);
      updateOrientation(localImuData.ax, localImuData.ay, localImuData.az,
                       localImuData.gx, localImuData.gy, localImuData.gz);
      
      if (xSemaphoreTake(featureDataMutex, pdMS_TO_TICKS(10))) {
        featurePacket.timestamp = localImuData.timestamp;
        featurePacket.ax = localImuData.ax;
        featurePacket.ay = localImuData.ay;
        featurePacket.az = localImuData.az;
        featurePacket.gx = localImuData.gx;
        featurePacket.gy = localImuData.gy;
        featurePacket.gz = localImuData.gz;
        featurePacket.rms = (int16_t)(getRMSValue() * 100.0f);
        featurePacket.pitch = (int16_t)(getPitch() * 100.0f);
        featurePacket.roll = (int16_t)(getRoll() * 100.0f);
        featurePacket.stepCount = getStepCount();
        
        FeaturePacket snapshot = featurePacket;
        xSemaphoreGive(featureDataMutex);
        
        sd_enqueue(&snapshot);
      }
      
      if (++updateCount % 50 == 0) {
        Serial.printf("T:%u A:%d,%d,%d | Pitch:%.1f° Roll:%.1f° Steps:%u\n",
          localImuData.timestamp,
          localImuData.ax, localImuData.ay, localImuData.az,
          getPitch(), getRoll(), getStepCount());
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ---------- BLE Task (10 Hz) ----------
void bleTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(BLE_NOTIFY_PERIOD_MS);
  uint32_t bleNotifyCount = 0;

  while (true) {
    if (isBLEConnected()) {
      FeaturePacket packet;
      
      if (xSemaphoreTake(featureDataMutex, pdMS_TO_TICKS(5))) {
        packet = featurePacket;
        xSemaphoreGive(featureDataMutex);

        NimBLECharacteristic* pChar = getBLECharacteristic();
        if (pChar) {
          pChar->setValue((uint8_t*)&packet, sizeof(packet));
          pChar->notify();
          
          if (++bleNotifyCount % 10 == 0) {
            Serial.printf("[BLE] Sent: Steps=%u, RMS=%d\n",
              packet.stepCount, packet.rms);
          }
        }
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ---------- SD Card Logging Task (buffered writes) ----------
void sdLoggingTask(void* parameter) {
  FeaturePacket logPacket;

  if (!sd_init()) {
    vTaskDelete(NULL);
    return;
  }

  xEventGroupSetBits(taskEventGroup, SD_READY_BIT);

  while (true) {
    if (sd_dequeue(&logPacket)) {
      if (!sd_writePacket(&logPacket)) {
        if (!sd_isReady()) {
          xEventGroupClearBits(taskEventGroup, SD_READY_BIT);
        }
      }
      
      SDCardStatus status = sd_getStatus();
      if (status.packetsWritten % 1000 == 0) {
        Serial.printf("[SD] %u packets, %u files, %u errors\n",
          status.packetsWritten, status.filesCreated, status.writeErrors);
      }
    } else {
      if (sd_isReady()) {
        sd_flush();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------- Initialize All Tasks and Event Groups ----------
void initTasks() {
  taskEventGroup = xEventGroupCreate();
  imuDataMutex = xSemaphoreCreateMutex();
  featureDataMutex = xSemaphoreCreateMutex();
  
  if (!taskEventGroup || !imuDataMutex || !featureDataMutex) {
    Serial.println("Failed to create synchronization objects!");
    return;
  }

  xTaskCreatePinnedToCore(imuSamplingTask, "IMU_Task", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(featureComputationTask, "Feature_Task", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(bleTask, "BLE_Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(sdLoggingTask, "SD_Task", 8192, NULL, 0, NULL, 0);
}
