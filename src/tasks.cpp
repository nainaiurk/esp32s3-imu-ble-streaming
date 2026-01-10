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

// Jitter tracking for IMU task
static volatile uint32_t imuMinPeriodUs = UINT32_MAX;
static volatile uint32_t imuMaxPeriodUs = 0;

// Mutexes for thread-safe access
SemaphoreHandle_t imuDataMutex;
SemaphoreHandle_t featureDataMutex;

// Event group for inter-task synchronization
EventGroupHandle_t taskEventGroup;

// ---------- IMU Sampling Task (50 Hz) ----------
void imuSamplingTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS);
  uint32_t lastSampleTimeUs = micros();
  const uint32_t expectedPeriodUs = IMU_SAMPLE_PERIOD_MS * 1000;

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    int16_t rawAccel[3], rawGyro[3], rawTemp;
    
    if (readIMUData(rawAccel, rawGyro, rawTemp)) {
      // Measure actual IMU sampling jitter in microseconds
      uint32_t currentSampleTimeUs = micros();
      uint32_t actualPeriodUs = currentSampleTimeUs - lastSampleTimeUs;
      
      if (actualPeriodUs < imuMinPeriodUs) {
        imuMinPeriodUs = actualPeriodUs;
      }
      if (actualPeriodUs > imuMaxPeriodUs) {
        imuMaxPeriodUs = actualPeriodUs;
      }
      
      lastSampleTimeUs = currentSampleTimeUs;
      
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
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ---------- BLE Task (10 Hz) ----------
void bleTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(BLE_NOTIFY_PERIOD_MS);
  uint32_t bleNotifyCount = 0;
  uint32_t loopCount = 0;

  while (true) {
    loopCount++;
    
    if (loopCount % 100 == 0) {  // Print status every 10 seconds
      DEBUG_LOG("[BLE Task] Loop=%u Connected=%d\n", loopCount, isBLEConnected());
    }
    
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
            DEBUG_LOG("[BLE] Sent: T=%u Steps=%u RMS=%d Pitch=%.1f Roll=%.1f\n",
              packet.timestamp, packet.stepCount, packet.rms,
              packet.pitch / 100.0f, packet.roll / 100.0f);
          }
        } else {
          DEBUG_LOG("[BLE] ERROR: pChar is NULL\n");
        }
      } else {
        DEBUG_LOG("[BLE] ERROR: Failed to acquire featureDataMutex\n");
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
        DEBUG_LOG("[SD] %u packets, %u files, %u errors\n",
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

// ---------- IMU Debug Task (1 Hz) ----------
void imuDebugTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // 1 Hz

  while (true) {
    ImuPacket debugData;
    
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5))) {
      debugData = imuPacket;
      xSemaphoreGive(imuDataMutex);
      
      int32_t minPeriodUs = imuMinPeriodUs;
      int32_t maxPeriodUs = imuMaxPeriodUs;
      
      const int32_t expectedPeriodUs = IMU_SAMPLE_PERIOD_MS * 1000;
      int32_t maxJitterUs = maxPeriodUs - expectedPeriodUs;
      int32_t minJitterUs = minPeriodUs - expectedPeriodUs;
      int32_t worstJitterUs = max(abs(maxJitterUs), abs(minJitterUs));
      
      Serial.printf("[IMU] Period min=%ld max=%ld | worst jitter=%ld µs\n", 
        minPeriodUs, maxPeriodUs, worstJitterUs);
      
      // Reset for next measurement window
      imuMinPeriodUs = UINT32_MAX;
      imuMaxPeriodUs = 0;
      
      if (DEBUG_LOG_ENABLE) {
        DEBUG_LOG("T:%u A:%d,%d,%d | Pitch:%.1f° Roll:%.1f° Steps:%u\n",
          debugData.timestamp,
          debugData.ax, debugData.ay, debugData.az,
          getPitch(), getRoll(), getStepCount());
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
  xTaskCreatePinnedToCore(imuDebugTask, "Debug_Task", 2048, NULL, 0, NULL, 0);
}
