#include "tasks.h"
#include "config.h"
#include "imu_sensor.h"
#include "feature_processing.h"
#include "ble_service.h"
#include "rtc_time.h"
#include "sd_card_data.h"
#include <Arduino.h>
#include <esp_timer.h>

// Global data packets
ImuPacket imuPacket;
FeaturePacket featurePacket;

// Jitter tracking for IMU task
static volatile uint32_t imuMinPeriodUs = UINT32_MAX;
static volatile uint32_t imuMaxPeriodUs = 0;

// IMU error tracking
static volatile uint8_t imuErrorCount = 0;
static volatile bool imuHealthy = true;
#define IMU_MAX_CONSECUTIVE_ERRORS 5

// Sample counters - single monotonic counter shared by IMU and Feature tasks
volatile uint32_t sampleIndex = 0;  // Starts at 0, incremented to 1 on first use
static uint32_t featureComputedCount = 0;  // Track computed features
static uint32_t featureEnqueuedCount = 0;  // Track enqueued features

// Mutexes for thread-safe access
SemaphoreHandle_t imuDataMutex;
SemaphoreHandle_t featureDataMutex;

// Event group for inter-task synchronization
EventGroupHandle_t taskEventGroup;

// ---------- IMU Sampling Task (50 Hz normal / 10 Hz low-power) ----------
void imuSamplingTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xFrequency = pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS);
  int64_t lastSampleTimeUs = esp_timer_get_time();
  const int64_t expectedPeriodUs = IMU_SAMPLE_PERIOD_MS * 1000;

  while (true) {
    // Dynamically adjust frequency based on power mode
    xFrequency = isInLowPowerMode() ? 
                 pdMS_TO_TICKS(1000 / LOW_POWER_IMU_RATE_HZ) : 
                 pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    int16_t rawAccel[3], rawGyro[3], rawTemp;
    
    if (readIMUData(rawAccel, rawGyro, rawTemp)) {
      // IMU read successful - reset error counter
      imuErrorCount = 0;
      if (!imuHealthy) {
        imuHealthy = true;
        DEBUG_LOG("[IMU] Recovered from error\n");
      }
      
      // Measure actual IMU sampling jitter using esp_timer (microsecond precision)
      int64_t currentSampleTimeUs = esp_timer_get_time();
      int64_t actualPeriodUs = currentSampleTimeUs - lastSampleTimeUs;
      
      if (actualPeriodUs < imuMinPeriodUs) {
        imuMinPeriodUs = actualPeriodUs;
      }
      if (actualPeriodUs > imuMaxPeriodUs) {
        imuMaxPeriodUs = actualPeriodUs;
      }
      
      lastSampleTimeUs = currentSampleTimeUs;
      
      if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5))) {
        time_t rtcSecs = getRTCTimestamp();
        
        imuPacket.sampleIndex = sampleIndex++;
        // Reset sample index to 1 every MAX_PACKETS_PER_FILE
        if (sampleIndex > MAX_PACKETS_PER_FILE) {
          sampleIndex = 1;
        }
        imuPacket.timestamp = rtcSecs;
        imuPacket.ax = rawAccel[0];
        imuPacket.ay = rawAccel[1];
        imuPacket.az = rawAccel[2];
        imuPacket.gx = rawGyro[0];
        imuPacket.gy = rawGyro[1];
        imuPacket.gz = rawGyro[2];
        imuPacket.temp = rawTemp;
        xSemaphoreGive(imuDataMutex);
      }
    } else {
      // IMU read failed - increment error counter
      imuErrorCount++;
      
      if (imuErrorCount == 1) {
        DEBUG_LOG("[IMU] ERROR: Read failed (attempt %u/%u)\n", 
          imuErrorCount, IMU_MAX_CONSECUTIVE_ERRORS);
      }
      
      if (imuErrorCount >= IMU_MAX_CONSECUTIVE_ERRORS) {
        // Critical failure
        imuHealthy = false;
        DEBUG_LOG("[IMU] CRITICAL: Failed %u consecutive reads - IMU may be disconnected\n", 
          imuErrorCount);
        // Wait a bit before retrying to avoid hammering I2C bus
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }
  }
}

// ---------- Feature Computation Task (50 Hz normal / 10 Hz low-power) ----------
void featureComputationTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xFrequency = pdMS_TO_TICKS(FEATURE_UPDATE_PERIOD_MS);
  uint16_t updateCount = 0;

  while (true) {
    // Dynamically adjust frequency based on power mode
    xFrequency = isInLowPowerMode() ? 
                 pdMS_TO_TICKS(1000 / LOW_POWER_FEATURE_RATE_HZ) : 
                 pdMS_TO_TICKS(FEATURE_UPDATE_PERIOD_MS);
    ImuPacket localImuData;
    
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(10))) {
      localImuData = imuPacket;
      xSemaphoreGive(imuDataMutex);
      
      featureComputedCount++;
      
      updateRMS(localImuData.ax, localImuData.ay, localImuData.az);
      detectStep(localImuData.ax, localImuData.ay, localImuData.az);
      updateOrientation(localImuData.ax, localImuData.ay, localImuData.az,
                       localImuData.gx, localImuData.gy, localImuData.gz);
      
      // Check motion for power saving mode
      detectPowerSavingMotion(localImuData.ax, localImuData.ay, localImuData.az);
      
      if (xSemaphoreTake(featureDataMutex, pdMS_TO_TICKS(10))) {
        featurePacket.sampleIndex = sampleIndex;  // Use same index as IMU (already incremented)
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
        featureEnqueuedCount++;
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ---------- BLE Task (50 Hz normal / 10 Hz low-power) ----------
void bleTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xFrequency = pdMS_TO_TICKS(BLE_NOTIFY_PERIOD_MS);
  uint32_t bleNotifyCount = 0;

  while (true) {
    // Dynamically adjust frequency based on power mode
    xFrequency = isInLowPowerMode() ? 
                 pdMS_TO_TICKS(1000 / LOW_POWER_FEATURE_RATE_HZ) : 
                 pdMS_TO_TICKS(BLE_NOTIFY_PERIOD_MS);
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
            DEBUG_LOG("[BLE] Notify T=%u Steps=%u RMS=%d\n",
              packet.timestamp, packet.stepCount, packet.rms);
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
  bool sdInitialized = false;
  uint32_t lastInitAttempt = 0;
  const uint32_t INIT_RETRY_MS = 1000;  // Retry every 1 second when disconnected

  while (true) {
    // Try to initialize/reinitialize SD card only if disconnected
    if (!sdInitialized) {
      uint32_t now = millis();
      if (now - lastInitAttempt >= INIT_RETRY_MS) {
        if (sd_init()) {
          sd_clearBuffer();  // Discard stale packets from buffer
          sdInitialized = true;
          xEventGroupSetBits(taskEventGroup, SD_READY_BIT);
          DEBUG_LOG("[SD] Initialized successfully\n");
        }
        lastInitAttempt = now;
      }
    }
    
    // Process queued packets if SD is ready
    if (sdInitialized) {
      if (sd_dequeue(&logPacket)) {
        if (!sd_writePacket(&logPacket)) {
          // Write failed - check if SD was removed
          if (!sd_isReady()) {
            sd_closeFile();
            sdInitialized = false;
            xEventGroupClearBits(taskEventGroup, SD_READY_BIT);
            DEBUG_LOG("[SD] Removed or became unavailable, waiting for reinsertion\n");
          }
        }
        
        SDCardStatus status = sd_getStatus();
        if (status.packetsWritten % 1000 == 0) {
          DEBUG_LOG("[SD] %u packets, %u files, %u errors\n",
            status.packetsWritten, status.filesCreated, status.writeErrors);
        }
      } else {
        // Idle: flush buffer and check SD health
        if (sd_isReady()) {
          sd_flush();
        } else {
          sd_closeFile();
          sdInitialized = false;
          xEventGroupClearBits(taskEventGroup, SD_READY_BIT);
          DEBUG_LOG("[SD] Card lost during idle, waiting for reinsertion\n");
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------- IMU Debug Task (1 Hz) ----------
void imuDebugTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  // 50 Hz - real-time orientation
  uint32_t statsCount = 0;
  uint32_t sdStatusCount = 0;  // Separate counter for SD status printing

  while (true) {
    ImuPacket debugData;
    
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5))) {
      debugData = imuPacket;
      xSemaphoreGive(imuDataMutex);
      
      // Print SYSTEM stats every 50 updates (once per second)
      if (statsCount++ % 50 == 0) {
        int32_t minPeriodUs = imuMinPeriodUs;
        int32_t maxPeriodUs = imuMaxPeriodUs;
        
        const int32_t expectedPeriodUs = IMU_SAMPLE_PERIOD_MS * 1000;
        int32_t maxJitterUs = maxPeriodUs - expectedPeriodUs;
        int32_t minJitterUs = minPeriodUs - expectedPeriodUs;
        int32_t worstJitterUs = max(abs(maxJitterUs), abs(minJitterUs));
        
        const char* imuStatus = imuHealthy ? "OK" : "FAIL";
        const char* bleStatus = isBLEConnected() ? "Connected" : "Disconnected";
        
        // Serial.printf("[SYSTEM] IMU=%s(%u) BLE=%s | Jitter=%ld µs\n", 
        //   imuStatus, imuErrorCount, bleStatus, worstJitterUs);
        
        // Print SD card status every 5 seconds
        if (++sdStatusCount >= 5) {
          sd_printStatus();
          
          // Show feature task metrics
          if (featureComputedCount != featureEnqueuedCount) {
            Serial.printf("[FEATURE] Computed:%u Enqueued:%u Missed:%u\n", 
              featureComputedCount, featureEnqueuedCount, 
              featureComputedCount - featureEnqueuedCount);
          }
          
          sdStatusCount = 0;
        }
        
        // Reset for next measurement window
        imuMinPeriodUs = UINT32_MAX;
        imuMaxPeriodUs = 0;
      }
      
      // Print real-time IMU and feature data every update (50 Hz)
      // Serial.printf("[IMU] T:%u A:%d,%d,%d G:%d,%d,%d\n",
      //   debugData.timestamp,
      //   debugData.ax, debugData.ay, debugData.az,
      //   debugData.gx, debugData.gy, debugData.gz);
      
      // Print real-time Feature packet with RTC timestamp
      if (xSemaphoreTake(featureDataMutex, pdMS_TO_TICKS(5))) {
        char timeStr[13];
        getTimeString(timeStr, sizeof(timeStr));
        
        Serial.printf("[%s] Index=%u RMS=%d Pitch:%.1f° Roll:%.1f° Steps:%u Mode=%s\n",
          timeStr,
          featurePacket.sampleIndex,
          featurePacket.rms,
          featurePacket.pitch / 100.0f,
          featurePacket.roll / 100.0f,
          featurePacket.stepCount,
          isInLowPowerMode() ? "LOW-POWER" : "NORMAL");
        xSemaphoreGive(featureDataMutex);
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
  xTaskCreatePinnedToCore(imuDebugTask, "Debug_Task", 8192, NULL, 0, NULL, 0);
}
