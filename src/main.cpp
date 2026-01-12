#include <Arduino.h>
#include "config.h"
#include "imu_sensor.h"
#include "feature_processing.h"
#include "ble_service.h"
#include "rtc_time.h"
#include "tasks.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  initRTC();
  initMPU6050();
  initFeatureProcessing();
  initBLEService();
  initTasks();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
