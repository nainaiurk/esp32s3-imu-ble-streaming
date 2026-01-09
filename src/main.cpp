#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>

#define DEVICE_NAME "ESP32-S3-BLE"
#define IMU_SAMPLE_RATE_HZ 50
#define IMU_SAMPLE_PERIOD_MS (1000 / IMU_SAMPLE_RATE_HZ)

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

NimBLECharacteristic* pCharacteristic;
bool deviceConnected = false;
uint32_t value = 0;
uint32_t lastNotify = 0;

// Binary packet structure
struct __attribute__((packed)) ImuPacket {
  uint32_t timestamp;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;
};

ImuPacket imuPacket;

SemaphoreHandle_t imuDataMutex;

/* ---------- Server Callbacks ---------- */
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) override {
    deviceConnected = true;
    NimBLEDevice::stopAdvertising();

    auto peer = pServer->getPeerInfo(0);
    pServer->updateConnParams(
      peer.getConnHandle(),
      12, 12, 0, 60
    );

    Serial.println("Client connected");
  }
  void onDisconnect(NimBLEServer* pServer) override {
    deviceConnected = false;
    NimBLEDevice::startAdvertising();
    Serial.println("Client disconnected");
  }
  
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) override {
    Serial.printf("MTU updated: %u\n", MTU);
  }
};

/* ---------- Characteristic Callbacks ---------- */
class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic) override {
    std::string rx = pCharacteristic->getValue();
    Serial.print("Received: ");
    Serial.println(rx.c_str());
  }
};

/* ---------- IMU Sampling Task ---------- */
void imuSamplingTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS);
  uint16_t sampleCount = 0;

  while (true) {
    // Read raw int16_t values directly - NO floats, NO conversion
    int16_t rawAccel[3], rawGyro[3], rawTemp;
    
    // Direct I2C read of raw sensor data (returns int16_t)
    Wire.beginTransmission(0x68);  // MPU6050 address
    Wire.write(0x3B);              // Starting register (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);  // Read 14 bytes
    
    // Read all sensor data in one burst (14 bytes = 7 int16_t values)
    rawAccel[0] = (Wire.read() << 8) | Wire.read();  // ACCEL_X
    rawAccel[1] = (Wire.read() << 8) | Wire.read();  // ACCEL_Y
    rawAccel[2] = (Wire.read() << 8) | Wire.read();  // ACCEL_Z
    rawTemp = (Wire.read() << 8) | Wire.read();      // TEMP
    rawGyro[0] = (Wire.read() << 8) | Wire.read();   // GYRO_X
    rawGyro[1] = (Wire.read() << 8) | Wire.read();   // GYRO_Y
    rawGyro[2] = (Wire.read() << 8) | Wire.read();   // GYRO_Z
    
    // Lock mutex and populate binary packet (pure int16_t, zero float math)
    if (xSemaphoreTake(imuDataMutex, portMAX_DELAY)) {
      imuPacket.timestamp = millis();
      imuPacket.ax = rawAccel[0];
      imuPacket.ay = rawAccel[1];
      imuPacket.az = rawAccel[2];
      imuPacket.gx = rawGyro[0];
      imuPacket.gy = rawGyro[1];
      imuPacket.gz = rawGyro[2];
      imuPacket.temp = rawTemp;
      xSemaphoreGive(imuDataMutex);
      
      // Print IMU data every 25 samples (every 0.5 seconds at 50 Hz)
      if (++sampleCount % 25 == 0) {
        Serial.printf("T:%u A:%d,%d,%d G:%d,%d,%d Temp:%d\n",
          imuPacket.timestamp,
          rawAccel[0], rawAccel[1], rawAccel[2],
          rawGyro[0], rawGyro[1], rawGyro[2],
          rawTemp);
      }
    }

    // Wait for next sample period (precise timing)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32-S3 IMU BLE System...");

  // Initialize I2C for IMU
  Wire.begin(20, 19);  // SDA=20, SCL=19
  Wire.setClock(400000);  // 400 kHz for faster burst reads
  
  Serial.println("Initializing MPU6050...");
  
  // Reset MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x80);  // Reset device
  Wire.endTransmission();
  delay(100);
  
  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up, use internal 8MHz oscillator
  Wire.endTransmission();
  delay(10);
  
  // Configure gyro: ±500°/s (FS_SEL=1)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x08);  // FS_SEL=1 (±500°/s)
  Wire.endTransmission();
  
  // Configure accel: ±8g (AFS_SEL=2)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x10);  // AFS_SEL=2 (±8g)
  Wire.endTransmission();
  
  // Set DLPF to ~20Hz bandwidth
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x04);  // DLPF_CFG=4 (20Hz)
  Wire.endTransmission();
  
  Serial.println("MPU6050 initialized!");

  // Create mutex for IMU data
  imuDataMutex = xSemaphoreCreateMutex();

  Serial.println("Starting NimBLE Server...");

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setMTU(247);  // Request larger MTU for future expansion

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pCharacteristic->setValue("Hello World");

  pService->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("BLE Server ready");

  // IMU sampling task
  xTaskCreatePinnedToCore(imuSamplingTask, "IMU_Task", 4096, NULL, 2, NULL, 1);

  Serial.print("IMU sampling at ");
  Serial.print(IMU_SAMPLE_RATE_HZ);
  Serial.println(" Hz");
}

void loop() {
  if (!deviceConnected) return;

  uint32_t now = millis();
  if (now - lastNotify >= IMU_SAMPLE_PERIOD_MS) {
    lastNotify = now;

    // Copy binary packet atomically
    ImuPacket packet;
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(10))) {
      packet = imuPacket;  // Fast memcpy, no heap
      xSemaphoreGive(imuDataMutex);

      // Send raw binary packet (18 bytes, no parsing, no heap)
      pCharacteristic->setValue((uint8_t*)&packet, sizeof(packet));
      pCharacteristic->notify();
    }
  }
}
