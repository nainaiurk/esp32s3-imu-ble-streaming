#include "sd_card_data.h"
#include "config.h"
#include <Arduino.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

// ----------- Ring Buffer Configuration -----------
#define SD_RING_BUFFER_SIZE 512  // 512 packets = ~10 sec buffer at 50 Hz

// Static allocation - avoids heap fragmentation for long-lived systems
static FeaturePacket sdRingBufferStorage[SD_RING_BUFFER_SIZE];
static uint32_t sdRingHead = 0;           // Write pointer
static uint32_t sdRingTail = 0;           // Read pointer
static uint32_t sdRingSize = 0;           // Current number of packets

// Ring buffer stats - production essentials
static uint32_t sdRingTotalEnqueued = 0;
static uint32_t sdRingTotalDequeued = 0;
static uint32_t sdRingDroppedPackets = 0;

// Ring buffer mutex for thread safety
static SemaphoreHandle_t sdRingBufferMutex = NULL;

// ----------- SD Card State Management -------------

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

// -------------- Ring Buffer Operations ---------------

static bool sd_ringBuffer_init() {
  if (sdRingBufferMutex == NULL) {
    sdRingBufferMutex = xSemaphoreCreateMutex();
    if (sdRingBufferMutex == NULL) return false;
  }

  sdRingHead = 0;
  sdRingTail = 0;
  sdRingSize = 0;
  sdRingTotalEnqueued = 0;
  sdRingTotalDequeued = 0;
  sdRingDroppedPackets = 0;

  return true;
}

/* Enqueue packet - DROP_OLDEST when full */
bool sd_enqueue(const FeaturePacket* packet) {
  if (!sdRingBufferMutex) return false;

  if (xSemaphoreTake(sdRingBufferMutex, 0) != pdTRUE) return false;

  // Buffer full - drop oldest packet
  if (sdRingSize >= SD_RING_BUFFER_SIZE) {
    sdRingDroppedPackets++;
    sdRingTail = (sdRingTail + 1) % SD_RING_BUFFER_SIZE;
    sdRingSize--;
  }

  sdRingBufferStorage[sdRingHead] = *packet;
  sdRingHead = (sdRingHead + 1) % SD_RING_BUFFER_SIZE;
  sdRingSize++;
  sdRingTotalEnqueued++;

  xSemaphoreGive(sdRingBufferMutex);
  return true;
}

/* Dequeue oldest packet */
bool sd_dequeue(FeaturePacket* packet) {
  if (!sdRingBufferMutex) return false;

  if (xSemaphoreTake(sdRingBufferMutex, 0) != pdTRUE) return false;

  if (sdRingSize == 0) {
    xSemaphoreGive(sdRingBufferMutex);
    return false;
  }

  *packet = sdRingBufferStorage[sdRingTail];
  sdRingTail = (sdRingTail + 1) % SD_RING_BUFFER_SIZE;
  sdRingSize--;
  sdRingTotalDequeued++;

  xSemaphoreGive(sdRingBufferMutex);
  return true;
}

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
    DEBUG_LOG("[SD] Error: Failed to create file: %s\n", sdLogFileName);
    return false;
  }

  // Update status
  sdPacketsInCurrentFile = 0;
  sdStatus.filesCreated++;
  strncpy(sdStatus.currentFilePath, sdLogFileName, sizeof(sdStatus.currentFilePath) - 1);
  
  DEBUG_LOG("[SD] Created new log file: %s\n", sdLogFileName);
  return true;
}

bool sd_init() {
  if (sdStatus.isInitialized) return true;

  if (!sd_ringBuffer_init()) return false;

  if (!SD_MMC.begin("/sdcard", true)) {
    sdStatus.isInitialized = false;
    sdStatus.isReady = false;
    return false;
  }

  if (!sd_createNewFile()) {
    sdStatus.isInitialized = false;
    sdStatus.isReady = false;
    return false;
  }

  sdStatus.isInitialized = true;
  sdStatus.isReady = true;
  sdStatus.packetsWritten = 0;
  sdStatus.writeErrors = 0;
  sdWriteErrorCount = 0;

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
    DEBUG_LOG("[SD] Write error #%u\n", sdStatus.writeErrors);

    // Check if too many consecutive errors
    if (sdWriteErrorCount >= 5) {
      DEBUG_LOG("[SD] Error: Too many write failures, marking SD as not ready\n");
      sdStatus.isReady = false;
      return false;
    }

    return false;
  }

  // Reset error counter on successful write
  sdWriteErrorCount = 0;
  sdStatus.packetsWritten++;
  sdPacketsInCurrentFile++;

  // Check if file rotation is needed (only explicit flush trigger)
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
    DEBUG_LOG("[SD] Closed log file: %u packets written\n", sdPacketsInCurrentFile);
  }

  // Create new file
  if (!sd_createNewFile()) {
    DEBUG_LOG("[SD] Error: Failed to create rotated log file\n");
    sdStatus.isReady = false;
    return false;
  }

  return true;
}
    
// ---------------Flush Current File-----------------
void sd_flush() {
  if (sdLogFile) {
    fflush(sdLogFile);
  }
}

// -------------------Close Current File-------------------
void sd_closeFile() {
  if (sdLogFile) {
    fflush(sdLogFile);
    fclose(sdLogFile);
    sdLogFile = NULL;

    DEBUG_LOG("[SD] Closed log file: %u packets total\n", sdPacketsInCurrentFile);
    sdPacketsInCurrentFile = 0;
  }
}

/* Get SD status */
SDCardStatus sd_getStatus() {
  // Update buffer stats before returning
  sdStatus.bufferStats.totalEnqueued = sdRingTotalEnqueued;
  sdStatus.bufferStats.totalDequeued = sdRingTotalDequeued;
  sdStatus.bufferStats.droppedPackets = sdRingDroppedPackets;
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

/* Diagnostic - print essential status only */
void sd_printStatus() {
  SDCardStatus status = sd_getStatus();

  Serial.println("\n===== SD Card Status =====");
  Serial.printf("Ready:            %s\n", status.isReady ? "Yes" : "No");
  Serial.printf("Packets Written:  %u\n", status.packetsWritten);
  Serial.printf("Write Errors:     %u\n", status.writeErrors);

  Serial.println("===== Ring Buffer =====");
  Serial.printf("Enqueued:  %u\n", status.bufferStats.totalEnqueued);
  Serial.printf("Dequeued:  %u\n", status.bufferStats.totalDequeued);
  Serial.printf("Dropped:   %u\n", status.bufferStats.droppedPackets);
  Serial.println("=======================\n");
}
