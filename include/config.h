#ifndef CONFIG_H
#define CONFIG_H

// Device configuration
#define DEVICE_NAME "ESP32-S3-BLE"

// Task timing configuration
#define IMU_SAMPLE_RATE_HZ 50
#define IMU_SAMPLE_PERIOD_MS (1000 / IMU_SAMPLE_RATE_HZ)
#define FEATURE_UPDATE_RATE_HZ 50
#define FEATURE_UPDATE_PERIOD_MS (1000 / FEATURE_UPDATE_RATE_HZ)
#define BLE_NOTIFY_RATE_HZ 10
#define BLE_NOTIFY_PERIOD_MS (1000 / BLE_NOTIFY_RATE_HZ)

// RMS configuration
#define RMS_WINDOW 25

// Step detection parameters
#define STEP_THRESHOLD 8000       // Threshold for dynamic acceleration magnitude
#define MIN_STEP_INTERVAL_MS 250  // Minimum time between steps (walking)
#define GRAVITY_ALPHA 0.98f       // Low-pass filter coefficient for gravity

// Orientation parameters
#define COMPLEMENTARY_ALPHA 0.98f  // Complementary filter coefficient (98% gyro, 2% accel)
#define GYRO_SCALE_500DPS 65.5f    // LSB/°/s for ±500°/s range
#define ACCEL_SCALE_8G 4096.0f     // LSB/g for ±8g range

// I2C configuration
#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 19
#define I2C_FREQ 400000
#define MPU6050_ADDR 0x68

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#endif
