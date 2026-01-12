#ifndef CONFIG_H
#define CONFIG_H

// Device configuration
#define DEVICE_NAME "ESP32-S3-BLE"

// Debug logging
#define DEBUG_LOG_ENABLE 0  // 0 = OFF, 1 = ON
#if DEBUG_LOG_ENABLE
  #define DEBUG_LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define DEBUG_LOG(fmt, ...) ((void)0)
#endif

// Task timing configuration
#define IMU_SAMPLE_RATE_HZ 50
#define IMU_SAMPLE_PERIOD_MS (1000 / IMU_SAMPLE_RATE_HZ)
#define FEATURE_UPDATE_RATE_HZ 50
#define FEATURE_UPDATE_PERIOD_MS (1000 / FEATURE_UPDATE_RATE_HZ)
#define BLE_NOTIFY_RATE_HZ 50
#define BLE_NOTIFY_PERIOD_MS (1000 / BLE_NOTIFY_RATE_HZ)

// RMS configuration
#define RMS_WINDOW 25

// Step detection parameters
#define STEP_THRESHOLD 10000      // Threshold for dynamic acceleration magnitude
#define MIN_STEP_INTERVAL_MS 250  // Minimum time between steps (walking)

// SD Card configuration
#define MAX_PACKETS_PER_FILE 500  // 10 seconds at 50 Hz

#define GRAVITY_ALPHA 0.98f       // Low-pass filter coefficient for gravity

// Orientation parameters
#define COMPLEMENTARY_ALPHA 0.80f  // Complementary filter coefficient (80% gyro, 20% accel)
#define GYRO_SCALE_500DPS 65.5f    // LSB/°/s for ±500°/s range
#define ACCEL_SCALE_4G 8192.0f     // LSB/g for ±4g range (improved precision for step detection)

// I2C configuration (IMU on Wire1)
#define I2C_SDA_PIN 38
#define I2C_SCL_PIN 39
#define I2C_FREQ 400000
#define MPU6050_ADDR 0x68

// RTC I2C configuration (DS3231 on Wire)
#define RTC_SDA_PIN 8
#define RTC_SCL_PIN 9

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// SD Card configuration
// #define SD_MMC_CLK_PIN    36   // Clock
// #define SD_MMC_CMD_PIN    35   // Command
// #define SD_MMC_D0_PIN     37   // Data 0
// #define SD_MMC_D1_PIN     -1   // Data 1 (not used, 1-bit mode)
// #define SD_MMC_D2_PIN     -1   // Data 2 (not used, 1-bit mode)
// #define SD_MMC_D3_PIN     -1   // Data 3 (pulled up externally)
// #define SD_MMC_MAX_FREQ   40000000  // 40 MHz for SD_MMC

#endif
