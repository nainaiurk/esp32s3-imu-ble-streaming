#include "imu_sensor.h"
#include "config.h"
#include <Wire.h>
#include <Arduino.h>

void initMPU6050() {
  Wire1.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire1.setClock(I2C_FREQ);
  
  Serial.println("Initializing IMU...");
  
  // Reset MPU6050
  Wire1.beginTransmission(MPU6050_ADDR);
  Wire1.write(0x6B);  // PWR_MGMT_1 register
  Wire1.write(0x80);  // Reset device
  Wire1.endTransmission();
  delay(100);
  
  // Wake up MPU6050
  Wire1.beginTransmission(MPU6050_ADDR);
  Wire1.write(0x6B);  // PWR_MGMT_1 register
  Wire1.write(0x00);  // Wake up, use internal 8MHz oscillator
  Wire1.endTransmission();
  delay(10);
  
  // Configure gyro
  Wire1.beginTransmission(MPU6050_ADDR);
  Wire1.write(0x1B);  // GYRO_CONFIG register
  Wire1.write(0x08);  // FS_SEL=1 (±500°/s)
  Wire1.endTransmission();
  
  // Configure accel
  Wire1.beginTransmission(MPU6050_ADDR);
  Wire1.write(0x1C);  // ACCEL_CONFIG register
  Wire1.write(0x08);  // AFS_SEL=1 (±4g) - improved precision for step detection
  Wire1.endTransmission();
  
  // Set DLPF to 20Hz bandwidth
  Wire1.beginTransmission(MPU6050_ADDR);
  Wire1.write(0x1A);  // CONFIG register
  Wire1.write(0x04);  // DLPF_CFG=4 (20Hz)
  Wire1.endTransmission();
  
  Serial.println("IMU initialized!");
}

bool readIMUData(int16_t rawAccel[3], int16_t rawGyro[3], int16_t& rawTemp) {
  //I2C read of raw sensor data
  Wire1.beginTransmission(MPU6050_ADDR);
  Wire1.write(0x3B);  // Starting register
  Wire1.endTransmission(false);
  Wire1.requestFrom(MPU6050_ADDR, 14, true);  // Request 14 bytes
  
  if (Wire1.available() < 14) {
    return false;
  }
  
  // Read all sensor data in one burst
  rawAccel[0] = (Wire1.read() << 8) | Wire1.read();  // ACCEL_X
  rawAccel[1] = (Wire1.read() << 8) | Wire1.read();  // ACCEL_Y
  rawAccel[2] = (Wire1.read() << 8) | Wire1.read();  // ACCEL_Z
  rawTemp = (Wire1.read() << 8) | Wire1.read();      // TEMP
  rawGyro[0] = (Wire1.read() << 8) | Wire1.read();   // GYRO_X
  rawGyro[1] = (Wire1.read() << 8) | Wire1.read();   // GYRO_Y
  rawGyro[2] = (Wire1.read() << 8) | Wire1.read();   // GYRO_Z
  
  return true;
}
