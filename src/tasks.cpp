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

// Queue for SD logging
QueueHandle_t sdLogQueue;

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
      
      // Queue feature packet for SD logging
      queueFeatureForSD(&featurePacket);
      
      // Signal that feature data is ready for BLE and SD tasks
      xEventGroupSetBits(taskEventGroup, FEATURE_READY_BIT);
      
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
  uint32_t bleNotifyCount = 0;

  // Wait for BLE to become available
  xEventGroupWaitBits(taskEventGroup, BLE_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(100));

  while (true) {
    // Wait for feature data to be ready (wait up to 200ms)
    xEventGroupWaitBits(taskEventGroup, FEATURE_READY_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(200));
    
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
          
          // Log BLE transmissions occasionally
          if (++bleNotifyCount % 10 == 0) {
            Serial.printf("[BLE] Notified client: T=%u, Steps=%u, RMS=%d\n",
              packet.timestamp, packet.stepCount, packet.rms);
          }
        }
      }
    } else {
      // Update BLE connected bit if disconnected
      xEventGroupClearBits(taskEventGroup, BLE_CONNECTED_BIT);
    }

    // Wait for next notification period
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* ---------- SD Card Logging Task (buffered writes) ---------- */
void sdLoggingTask(void* parameter) {
  FeaturePacket logPacket;
  uint32_t queueReceiveCount = 0;

  // Initialize SD card
  if (!sd_init()) {
    Serial.println("[Task] SD initialization failed, task exiting");
    vTaskDelete(NULL);
    return;
  }

  // Signal SD is ready
  xEventGroupSetBits(taskEventGroup, SD_READY_BIT);
  Serial.println("[Task] SD Logging task started, queue listening...");

  while (true) {
    // Receive feature packets from queue (wait up to 500ms)
    if (xQueueReceive(sdLogQueue, &logPacket, pdMS_TO_TICKS(500))) {
      queueReceiveCount++;
      
      // Write packet using SD module
      if (!sd_writePacket(&logPacket)) {
        Serial.printf("[Task] Write failed for packet %u\n", queueReceiveCount);
        
        // Check if SD became unavailable
        if (!sd_isReady()) {
          Serial.println("[Task] SD card no longer ready, clearing SD_READY_BIT");
          xEventGroupClearBits(taskEventGroup, SD_READY_BIT);
        }
      }
      
      // Print status every 1000 packets (~20 seconds at 50 Hz)
      if (queueReceiveCount % 1000 == 0) {
        SDCardStatus status = sd_getStatus();
        Serial.printf("[Task] SD Progress: %u packets, %u files, %u errors\n",
          status.packetsWritten, status.filesCreated, status.writeErrors);
      }
    } else {
      // Queue timeout - no data received for 500ms
      // Periodically flush data to SD
      if (sd_isReady()) {
        sd_flush();
      }
    }

    // Allow other tasks to run
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ---------- Utility Function: Queue Feature for SD Logging ---------- */
void queueFeatureForSD(const FeaturePacket* packet) {
  // Check if SD task is ready
  EventBits_t bits = xEventGroupGetBits(taskEventGroup);
  
  if (bits & SD_READY_BIT) {
    // Try to send to queue (non-blocking)
    BaseType_t result = xQueueSendToBack(sdLogQueue, (void*)packet, 0);
    
    if (result != pdPASS) {
      // Queue is full, signal buffer full condition
      xEventGroupSetBits(taskEventGroup, BUFFER_FULL_BIT);
      Serial.println("[Queue] SD logging queue is full!");
    }
  }
}

/* ---------- Initialize All Tasks, Queues, and Event Groups ---------- */
void initTasks() {
  // Create event group for inter-task synchronization
  taskEventGroup = xEventGroupCreate();
  if (!taskEventGroup) {
    Serial.println("Failed to create event group!");
    return;
  }

  // Create queue for SD logging
  sdLogQueue = xQueueCreate(SD_LOG_QUEUE_SIZE, sizeof(FeaturePacket));
  if (!sdLogQueue) {
    Serial.println("Failed to create SD logging queue!");
    return;
  }

  // Create mutexes for data sharing
  imuDataMutex = xSemaphoreCreateMutex();
  featureDataMutex = xSemaphoreCreateMutex();
  
  if (!imuDataMutex || !featureDataMutex) {
    Serial.println("Failed to create mutexes!");
    return;
  }

  // Create FreeRTOS tasks with appropriate priorities and core affinity
  
  // Task 1: IMU sampling task (highest priority, core 1)
  // Needs to run on precise intervals without interruption
  xTaskCreatePinnedToCore(
    imuSamplingTask,        // Task function
    "IMU_Task",             // Task name
    4096,                   // Stack size
    NULL,                   // Parameter
    3,                      // Priority (higher number = higher priority)
    NULL,                   // Task handle
    1                       // Core (1 = second core, usually reserved for non-BLE)
  );
  
  // Task 2: Feature computation task (medium priority, core 1)
  // Depends on IMU data, feeds both BLE and SD tasks
  xTaskCreatePinnedToCore(
    featureComputationTask,
    "Feature_Task",
    8192,                   // Larger stack for feature processing
    NULL,
    2,                      // Medium priority
    NULL,
    1                       // Core 1
  );
  
  // Task 3: BLE notification task (lower priority, core 0)
  // Core 0 is typically reserved for BLE/WiFi stack
  xTaskCreatePinnedToCore(
    bleTask,
    "BLE_Task",
    4096,
    NULL,
    1,                      // Lower priority (non-critical notifications)
    NULL,
    0                       // Core 0 (BLE stack)
  );
  
  // Task 4: SD card logging task (lowest priority, core 0)
  // Can tolerate variable latency for buffered writes
  xTaskCreatePinnedToCore(
    sdLoggingTask,
    "SD_Task",
    8192,                   // Larger stack for file I/O
    NULL,
    0,                      // Lowest priority
    NULL,
    0                       // Core 0
  );

  // Print task configuration
  Serial.println("\n========== RTOS Task Configuration ==========");
  Serial.printf("Task 1: IMU Sampling at %d Hz (20 ms period)\n", IMU_SAMPLE_RATE_HZ);
  Serial.printf("Task 2: Feature Computation at %d Hz\n", FEATURE_UPDATE_RATE_HZ);
  Serial.printf("Task 3: BLE Notifications at %d Hz\n", BLE_NOTIFY_RATE_HZ);
  Serial.println("Task 4: SD Card Logging (buffered, file rotation enabled)");
  Serial.printf("SD Queue: %d max packets\n", SD_LOG_QUEUE_SIZE);
  Serial.println("==========================================\n");
}
