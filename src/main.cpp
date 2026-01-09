#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>

#define DEVICE_NAME "ESP32-S3-BLE"
#define IMU_SAMPLE_RATE_HZ 50
#define IMU_SAMPLE_PERIOD_MS (1000 / IMU_SAMPLE_RATE_HZ)
#define RMS_WINDOW 25
#define FEATURE_UPDATE_RATE_HZ 50
#define FEATURE_UPDATE_PERIOD_MS (1000 / FEATURE_UPDATE_RATE_HZ)
#define BLE_NOTIFY_RATE_HZ 10
#define BLE_NOTIFY_PERIOD_MS (1000 / BLE_NOTIFY_RATE_HZ)

// Step detection parameters
#define STEP_THRESHOLD 8000       // Threshold for dynamic acceleration magnitude
#define MIN_STEP_INTERVAL_MS 250  // Minimum time between steps (walking)
#define GRAVITY_ALPHA 0.98f       // Low-pass filter coefficient for gravity

// Orientation parameters
#define COMPLEMENTARY_ALPHA 0.98f  // Complementary filter coefficient (98% gyro, 2% accel)
#define GYRO_SCALE_500DPS 65.5f    // LSB/°/s for ±500°/s range
#define ACCEL_SCALE_8G 4096.0f     // LSB/g for ±8g range

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

NimBLECharacteristic* pCharacteristic;
bool deviceConnected = false;
uint32_t value = 0;
uint32_t lastNotify = 0;

int32_t accSqSum = 0;
uint16_t rmsCount = 0;
float rmsValue = 0;

// Step detection variables
float gravityX = 0, gravityY = 0, gravityZ = 0;  // Estimated gravity vector
int32_t prevDynMagSq = 0;
int32_t lastDynamicMagSq = 0;
uint32_t stepCount = 0;
uint32_t lastStepTime = 0;

// Orientation variables
float pitch = 0.0f;  // Rotation around X-axis (forward/backward tilt)
float roll = 0.0f;   // Rotation around Y-axis (left/right tilt)
uint32_t lastOrientationUpdate = 0;

// Binary packet structure for raw IMU data
struct __attribute__((packed)) ImuPacket {
  uint32_t timestamp;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;
};

// Extended packet structure with computed features
struct __attribute__((packed)) FeaturePacket {
  uint32_t timestamp;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t rms;    // RMS * 100 (scaled)
  int16_t pitch;  // degrees * 100 (scaled)
  int16_t roll;   // degrees * 100 (scaled)
  uint32_t stepCount;
};

ImuPacket imuPacket;
FeaturePacket featurePacket;

SemaphoreHandle_t imuDataMutex;
SemaphoreHandle_t featureDataMutex;

/* ---------- RMS Calculation Function ---------- */
void updateRMS(int16_t ax, int16_t ay, int16_t az) {
    int32_t magSq = (int32_t)ax*ax + (int32_t)ay*ay + (int32_t)az*az;
    accSqSum += magSq;
    rmsCount++;

    if (rmsCount >= RMS_WINDOW) {
        rmsValue = sqrt((float)accSqSum / RMS_WINDOW);
        accSqSum = 0;
        rmsCount = 0;
    }
}

/* ---------- Step Detection with Gravity Removal ---------- */
void detectStep(int16_t ax, int16_t ay, int16_t az) {
    // Convert to float
    float fx = (float)ax;
    float fy = (float)ay;
    float fz = (float)az;
    
    // Low-pass filter to estimate gravity (slowly adapts to orientation)
    gravityX = GRAVITY_ALPHA * gravityX + (1.0f - GRAVITY_ALPHA) * fx;
    gravityY = GRAVITY_ALPHA * gravityY + (1.0f - GRAVITY_ALPHA) * fy;
    gravityZ = GRAVITY_ALPHA * gravityZ + (1.0f - GRAVITY_ALPHA) * fz;
    
    // Remove gravity to get dynamic (linear) acceleration
    float dynX = fx - gravityX;
    float dynY = fy - gravityY;
    float dynZ = fz - gravityZ;
    
    // Calculate magnitude squared of dynamic acceleration
    int32_t dynMagSq = (int32_t)(dynX*dynX + dynY*dynY + dynZ*dynZ);
    
    uint32_t now = millis();
    
    // Three-point peak detection: prev < last > current (local maximum)
    if (prevDynMagSq < lastDynamicMagSq &&
        lastDynamicMagSq > dynMagSq &&
        lastDynamicMagSq > STEP_THRESHOLD*STEP_THRESHOLD &&
        (now - lastStepTime) >= MIN_STEP_INTERVAL_MS) {

        stepCount++;
        lastStepTime = now;
        
        Serial.printf("Step #%u detected! Dynamic mag: %.1f\n", 
                      stepCount, sqrt((float)lastDynamicMagSq));
    }
    prevDynMagSq = lastDynamicMagSq;
    lastDynamicMagSq = dynMagSq;
}

/* ---------- Orientation Estimation (Complementary Filter) ---------- */
void updateOrientation(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    uint32_t now = millis();
    float dt = (now - lastOrientationUpdate) / 1000.0f;  // Convert to seconds
    lastOrientationUpdate = now;
    
    // Skip first iteration (dt invalid)
    if (dt > 1.0f || dt <= 0.0f) {
        dt = 0.02f;  // Default to 50Hz
    }
    
    // Convert accelerometer to g's
    float ax_g = ax / ACCEL_SCALE_8G;
    float ay_g = ay / ACCEL_SCALE_8G;
    float az_g = az / ACCEL_SCALE_8G;
    
    // Calculate pitch and roll from accelerometer (in degrees)
    // accelPitch = atan2(ay, sqrt(ax^2 + az^2))
    // accelRoll  = atan2(-ax, az)
    float accelPitch = atan2(ay_g, sqrt(ax_g*ax_g + az_g*az_g)) * 180.0f / PI;
    float accelRoll = atan2(-ax_g, az_g) * 180.0f / PI;
    
    // Convert gyroscope to °/s and integrate
    float gx_dps = gx / GYRO_SCALE_500DPS;
    float gy_dps = gy / GYRO_SCALE_500DPS;
    
    // Integrate gyro rates to get angles (gyro measures rotation rate)
    float gyroPitch = pitch + gx_dps * dt;
    float gyroRoll = roll - gy_dps * dt;  // Negative because of axis orientation
    
    // Complementary filter: 98% gyro (short-term accuracy), 2% accel (long-term stability)
    pitch = COMPLEMENTARY_ALPHA * gyroPitch + (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;
    roll = COMPLEMENTARY_ALPHA * gyroRoll + (1.0f - COMPLEMENTARY_ALPHA) * accelRoll;
}

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

/* ---------- IMU Sampling Task (50 Hz) ---------- */
void imuSamplingTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS);

  while (true) {
    int16_t rawAccel[3], rawGyro[3], rawTemp;
    
    // Direct I2C read of raw sensor data (returns int16_t)
    Wire.beginTransmission(0x68);  // MPU6050 address
    Wire.write(0x3B);              // Starting register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);  // Read 14 bytes
    
    // Read all sensor data in one burst
    rawAccel[0] = (Wire.read() << 8) | Wire.read();  // ACCEL_X
    rawAccel[1] = (Wire.read() << 8) | Wire.read();  // ACCEL_Y
    rawAccel[2] = (Wire.read() << 8) | Wire.read();  // ACCEL_Z
    rawTemp = (Wire.read() << 8) | Wire.read();      // TEMP
    rawGyro[0] = (Wire.read() << 8) | Wire.read();   // GYRO_X
    rawGyro[1] = (Wire.read() << 8) | Wire.read();   // GYRO_Y
    rawGyro[2] = (Wire.read() << 8) | Wire.read();   // GYRO_Z
    
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

    // Wait for next sample period (precise timing)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* ---------- Feature Computation Task (10 Hz) ---------- */
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
        featurePacket.rms = (int16_t)(rmsValue * 100.0f);      // Scale by 100
        featurePacket.pitch = (int16_t)(pitch * 100.0f);       // Scale by 100
        featurePacket.roll = (int16_t)(roll * 100.0f);         // Scale by 100
        featurePacket.stepCount = stepCount;
        xSemaphoreGive(featureDataMutex);
      }
      
      // Print status every 25 updates (every 0.5 seconds at 50 Hz)
      if (++updateCount % 25 == 0) {
        Serial.printf("T:%u A:%d,%d,%d G:%d,%d,%d | Pitch:%.1f° Roll:%.1f° Steps:%u RMS:%.1f\n",
          localImuData.timestamp,
          localImuData.ax, localImuData.ay, localImuData.az,
          localImuData.gx, localImuData.gy, localImuData.gz,
          pitch, roll, stepCount, rmsValue);
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
    if (deviceConnected) {
      FeaturePacket packet;
      
      // Copy feature packet atomically
      if (xSemaphoreTake(featureDataMutex, pdMS_TO_TICKS(10))) {
        packet = featurePacket;
        xSemaphoreGive(featureDataMutex);

        // Send feature packet via BLE
        pCharacteristic->setValue((uint8_t*)&packet, sizeof(packet));
        pCharacteristic->notify();
      }
    }

    // Wait for next notification period
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

  // Create mutexes for data sharing
  imuDataMutex = xSemaphoreCreateMutex();
  featureDataMutex = xSemaphoreCreateMutex();

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

void loop() {
  // Main loop is minimal - all work done in FreeRTOS tasks
  // Handle any non-time-critical operations here if needed
  vTaskDelay(pdMS_TO_TICKS(1000));  // Yield to other tasks
}
