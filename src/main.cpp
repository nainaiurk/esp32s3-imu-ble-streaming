#include <Arduino.h>
#include "config.h"
#include "imu_sensor.h"
#include "feature_processing.h"
#include "ble_service.h"
#include "tasks.h"

// Forward declaration for BLE callbacks
extern EventGroupHandle_t taskEventGroup;

// BLE server callbacks to update task synchronization
class CustomBLEServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
    Serial.println("[BLE] Client connected");
    xEventGroupSetBits(taskEventGroup, BLE_CONNECTED_BIT);
  }

  void onDisconnect(NimBLEServer* pServer) override {
    Serial.println("[BLE] Client disconnected");
    xEventGroupClearBits(taskEventGroup, BLE_CONNECTED_BIT);
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give serial time to initialize
  
  Serial.println("\n========================================");
  Serial.println("ESP32-S3 IMU BLE SD Card RTOS System");
  Serial.println("========================================\n");
  
  // Initialize hardware
  Serial.println("[Setup] Initializing MPU6050...");
  initMPU6050();
  
  Serial.println("[Setup] Initializing feature processing...");
  initFeatureProcessing();
  
  Serial.println("[Setup] Initializing BLE service...");
  initBLEService();
  
  // Set BLE callbacks for connection events
  NimBLEServer* pServer = NimBLEDevice::getServer();
  if (pServer) {
    pServer->setCallbacks(new CustomBLEServerCallbacks());
  }
  
  // Initialize RTOS tasks, queues, and event groups
  Serial.println("[Setup] Initializing RTOS tasks...");
  initTasks();
  
  Serial.println("[Setup] System ready - all tasks running\n");
}

void loop() {
  // FreeRTOS handles task scheduling
  // This loop just yields to allow other tasks to run
  vTaskDelay(pdMS_TO_TICKS(1000));
}
