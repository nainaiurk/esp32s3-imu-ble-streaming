#include <Arduino.h>
#include "config.h"
#include "imu_sensor.h"
#include "feature_processing.h"
#include "ble_service.h"
#include "tasks.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32-S3 IMU BLE System...");

  // Initialize IMU sensor
  initMPU6050();

  // Initialize feature processing modules
  initFeatureProcessing();

  // Initialize BLE service
  initBLEService();

  // Initialize and start FreeRTOS tasks
  initTasks();
}

void loop() {
  // Main loop is minimal - all work done in FreeRTOS tasks
  // Handle any non-time-critical operations here if needed
  vTaskDelay(pdMS_TO_TICKS(1000));  // Yield to other tasks
}
