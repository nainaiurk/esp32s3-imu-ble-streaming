#include "sd_card_data.h"
#include "config.h"
#include <Arduino.h>
#include <SD_MMC.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

//SD Card State Management

static FILE* sdLogFile = NULL;
static char sdLogFileName[128];
static SDCardStatus sdStatus = {
  .isInitialized = false,
  .isReady = false,
  .packetsWritten = 0,
  .filesCreated = 0,
  .writeErrors = 0,
  .currentFilePath = {0}
};

static uint32_t sdPacketsInCurrentFile = 0;
static const uint32_t MAX_PACKETS_PER_FILE = 60000;  // ~10 min at 50 Hz
static uint8_t sdWriteErrorCount = 0;

// -------------------Generate Log Filename-------------------
void sd_getFileName(char* buffer, size_t bufferSize) {
  if (!buffer || bufferSize < 32) {
    return;
  }

  time_t now = time(NULL);
  struct tm* timeinfo = localtime(&now);
  
  snprintf(buffer, bufferSize, "/sdcard/imu_log_%04d%02d%02d_%02d%02d%02d.bin",
    timeinfo->tm_year + 1900,
    timeinfo->tm_mon + 1,
    timeinfo->tm_mday,
    timeinfo->tm_hour,
    timeinfo->tm_min,
    timeinfo->tm_sec);
}

// ------------------Create New Log File-----------------
static bool sd_createNewFile() {
  sd_getFileName(sdLogFileName, sizeof(sdLogFileName));
  
  // Open file for binary writing
  sdLogFile = fopen(sdLogFileName, "wb");
  if (!sdLogFile) {
    Serial.printf("[SD] Error: Failed to create file: %s\n", sdLogFileName);
    return false;
  }

  // Update status
  sdPacketsInCurrentFile = 0;
  sdStatus.filesCreated++;
  strncpy(sdStatus.currentFilePath, sdLogFileName, sizeof(sdStatus.currentFilePath) - 1);
  
  Serial.printf("[SD] Created new log file: %s\n", sdLogFileName);
  return true;
}

// ------------------Initialize SD Card----------------------
bool sd_init() {
  if (sdStatus.isInitialized) {
    Serial.println("[SD] Already initialized");
    return true;
  }

  // Initialize SD_MMC
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[SD] Error: Failed to initialize SD_MMC interface");
    sdStatus.isInitialized = false;
    sdStatus.isReady = false;
    return false;
  }

  Serial.println("[SD] SD_MMC interface initialized successfully");

  // Check card size
  uint64_t cardSize = SD_MMC.cardSize();
  Serial.printf("[SD] Card size: %llu MB\n", cardSize / (1024 * 1024));

  // Create first log file
  if (!sd_createNewFile()) {
    Serial.println("[SD] Error: Failed to create initial log file");
    sdStatus.isInitialized = false;
    sdStatus.isReady = false;
    return false;
  }

  sdStatus.isInitialized = true;
  sdStatus.isReady = true;
  sdStatus.packetsWritten = 0;
  sdStatus.writeErrors = 0;
  sdWriteErrorCount = 0;

  Serial.println("[SD] Initialization complete");
  return true;
}

// ---------------Check if SD Card is Ready----------------- 
bool sd_isReady() {
  return sdStatus.isReady && sdStatus.isInitialized && (sdLogFile != NULL);
}

// ---------------Write Feature Packet to SD-----------------
bool sd_writePacket(const FeaturePacket* packet) {
  if (!packet || !sd_isReady()) {
    return false;
  }

  // Write packet to file
  size_t bytesWritten = fwrite(packet, sizeof(FeaturePacket), 1, sdLogFile);

  if (bytesWritten != 1) {
    sdWriteErrorCount++;
    sdStatus.writeErrors++;
    Serial.printf("[SD] Write error #%u\n", sdStatus.writeErrors);

    // Check if too many consecutive errors
    if (sdWriteErrorCount >= 5) {
      Serial.println("[SD] Error: Too many write failures, marking SD as not ready");
      sdStatus.isReady = false;
      return false;
    }

    return false;
  }

  // Reset error counter on successful write
  sdWriteErrorCount = 0;
  sdStatus.packetsWritten++;
  sdPacketsInCurrentFile++;

  // Flush every 100 packets (~2 seconds at 50 Hz)
  if (sdPacketsInCurrentFile % 100 == 0) {
    sd_flush();
  }

  // Check if file rotation is needed
  if (sdPacketsInCurrentFile >= MAX_PACKETS_PER_FILE) {
    if (!sd_rotateFile()) {
      sdStatus.isReady = false;
      return false;
    }
  }

  return true;
}

// -------------------Rotate to New Log File------------------
bool sd_rotateFile() {
  if (sdLogFile) {
    // Flush and close current file
    fflush(sdLogFile);
    fclose(sdLogFile);
    sdLogFile = NULL;
    Serial.printf("[SD] Closed log file: %u packets written\n", sdPacketsInCurrentFile);
  }

  // Create new file
  if (!sd_createNewFile()) {
    Serial.println("[SD] Error: Failed to create rotated log file");
    sdStatus.isReady = false;
    return false;
  }

  return true;
}
    
// ---------------Flush Current File-----------------
void sd_flush() {
  if (sdLogFile) {
    fflush(sdLogFile);
    Serial.printf("[SD] Flushed %u packets to disk\n", sdPacketsInCurrentFile);
  }
}

// -------------------Close Current File-------------------
void sd_closeFile() {
  if (sdLogFile) {
    fflush(sdLogFile);
    fclose(sdLogFile);
    sdLogFile = NULL;
    
    Serial.printf("[SD] Closed log file: %u packets total\n", sdPacketsInCurrentFile);
    sdPacketsInCurrentFile = 0;
  }
}

// -------------------Get SD Card Status-------------------
SDCardStatus sd_getStatus() {
  return sdStatus;
}

// -------------------Get Available Space on SD Card-------------------
uint64_t sd_getAvailableSpace() {
  // Estimate based on card size and used space
  if (!sdStatus.isInitialized) {
    return 0;
  }

  uint64_t cardSize = SD_MMC.cardSize();
  uint64_t estimatedUsed = sdStatus.packetsWritten * sizeof(FeaturePacket);
  
  if (estimatedUsed >= cardSize) {
    return 0;
  }

  return cardSize - estimatedUsed;
}

// -------------------Get Total Packet Count-------------------
uint32_t sd_getPacketCount() {
  return sdStatus.packetsWritten;
}

// -------------------Diagnostic Function: Print Status-------------------
void sd_printStatus() {
  SDCardStatus status = sd_getStatus();
  
  Serial.println("\n========== SD Card Status ==========");
  Serial.printf("Initialized:      %s\n", status.isInitialized ? "Yes" : "No");
  Serial.printf("Ready:            %s\n", status.isReady ? "Yes" : "No");
  Serial.printf("Packets Written:  %u\n", status.packetsWritten);
  Serial.printf("Files Created:    %u\n", status.filesCreated);
  Serial.printf("Write Errors:     %u\n", status.writeErrors);
  Serial.printf("Current File:     %s\n", status.currentFilePath);
  
  uint64_t availSpace = sd_getAvailableSpace();
  Serial.printf("Available Space:  %llu MB\n", availSpace / (1024 * 1024));
  Serial.println("=====================================\n");
}
