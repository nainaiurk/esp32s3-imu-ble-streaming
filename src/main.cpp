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
  delay(1000);
  
  initMPU6050();
  initFeatureProcessing();
  initBLEService();
  
  NimBLEServer* pServer = NimBLEDevice::getServer();
  if (pServer) {
    pServer->setCallbacks(new CustomBLEServerCallbacks());
  }
  
  initTasks();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
