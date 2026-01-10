#include "sd_card_data.h"
#include "config.h"
#include <Arduino.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

// ============ Ring Buffer Configuration ============
#define SD_RING_BUFFER_SIZE 512

// Static allocation - avoids heap fragmentation
static FeaturePacket sdRingBufferStorage[SD_RING_BUFFER_SIZE];

// ============ Ring Buffer Implementation ============
typedef struct {
  FeaturePacket* buffer;
  uint32_t head;
  uint32_t tail;
  uint32_t size;
  uint32_t capacity;
  SDBackpressurePolicy policy;
  SDRingBufferStats stats;
} SDRingBuffer;

static SDRingBuffer sdRingBuffer = {
  .buffer = sdRingBufferStorage,
  .head = 0,
  .tail = 0,
  .size = 0,
  .capacity = SD_RING_BUFFER_SIZE,
  .policy = SD_BACKPRESSURE_DROP_OLDEST,
  .stats = {.totalEnqueued = 0, .totalDequeued = 0, .peakQueueDepth = 0, 
            .droppedPackets = 0, .mutexContentions = 0}
};

static SemaphoreHandle_t sdRingBufferMutex = NULL;

// ============ SD Card State ============
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
static const uint32_t MAX_PACKETS_PER_FILE = 60000;
static uint8_t sdWriteErrorCount = 0;

// ============ Ring Buffer Operations ============

static bool sd_ringBuffer_init() {
  if (sdRingBufferMutex == NULL) {
    sdRingBufferMutex = xSemaphoreCreateMutex();
    if (sdRingBufferMutex == NULL) {
      Serial.println("[RB] Failed to create mutex");
      return false;
    }
  }

  sdRingBuffer.head = 0;
  sdRingBuffer.tail = 0;
  sdRingBuffer.size = 0;
  memset(&sdRingBuffer.stats, 0, sizeof(SDRingBufferStats));

  Serial.printf("[RB] Ring buffer: %u packets, %zu bytes (static)\n",
    sdRingBuffer.capacity, sdRingBuffer.capacity * sizeof(FeaturePacket));
  return true;
}

/* Enqueue packet with non-blocking mutex and backpressure */
bool sd_enqueue(const FeaturePacket* packet) {
  if (!sdRingBufferMutex) return false;

  if (xSemaphoreTake(sdRingBufferMutex, 0) != pdTRUE) {
    sdRingBuffer.stats.mutexContentions++;
    return false;
  }

  if (sdRingBuffer.size == sdRingBuffer.capacity) {
    sdRingBuffer.stats.droppedPackets++;

    if (sdRingBuffer.policy == SD_BACKPRESSURE_DROP_NEWEST) {
      xSemaphoreGive(sdRingBufferMutex);
      return false;
    }

    sdRingBuffer.tail = (sdRingBuffer.tail + 1) % sdRingBuffer.capacity;
    sdRingBuffer.size--;
  }

  sdRingBuffer.buffer[sdRingBuffer.head] = *packet;
  sdRingBuffer.head = (sdRingBuffer.head + 1) % sdRingBuffer.capacity;
  sdRingBuffer.size++;
  sdRingBuffer.stats.totalEnqueued++;

  if (sdRingBuffer.size > sdRingBuffer.stats.peakQueueDepth) {
    sdRingBuffer.stats.peakQueueDepth = sdRingBuffer.size;
  }

  xSemaphoreGive(sdRingBufferMutex);
  return true;
}

/* Dequeue oldest packet - non-blocking */
bool sd_dequeue(FeaturePacket* packet) {
  if (!sdRingBufferMutex) return false;

  if (xSemaphoreTake(sdRingBufferMutex, 0) != pdTRUE) {
    sdRingBuffer.stats.mutexContentions++;
    return false;
  }

  if (sdRingBuffer.size == 0) {
    xSemaphoreGive(sdRingBufferMutex);
    return false;
  }

  *packet = sdRingBuffer.buffer[sdRingBuffer.tail];
  sdRingBuffer.tail = (sdRingBuffer.tail + 1) % sdRingBuffer.capacity;
  sdRingBuffer.size--;
  sdRingBuffer.stats.totalDequeued++;

  xSemaphoreGive(sdRingBufferMutex);
  return true;
}

/* Set backpressure policy */
void sd_setBackpressurePolicy(SDBackpressurePolicy p) {
  sdRingBuffer.policy = p;
  const char* names[] = {"DROP_OLDEST", "DROP_NEWEST"};
  Serial.printf("[RB] Backpressure: %s\n", names[p]);
}

// ============ SD File Operations ============

void sd_getFileName(char* buffer, size_t bufferSize) {
  if (!buffer || bufferSize < 32) return;

  time_t now = time(NULL);
  struct tm* tm = localtime(&now);

  snprintf(buffer, bufferSize, "/sdcard/imu_log_%04d%02d%02d_%02d%02d%02d.bin",
    tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
    tm->tm_hour, tm->tm_min, tm->tm_sec);
}

static bool sd_createNewFile() {
  sd_getFileName(sdLogFileName, sizeof(sdLogFileName));

  sdLogFile = fopen(sdLogFileName, "wb");
  if (!sdLogFile) {
    Serial.printf("[SD] Failed to create: %s\n", sdLogFileName);
    return false;
  }

  sdPacketsInCurrentFile = 0;
  sdStatus.filesCreated++;
  strncpy(sdStatus.currentFilePath, sdLogFileName, sizeof(sdStatus.currentFilePath) - 1);

  Serial.printf("[SD] Created: %s\n", sdLogFileName);
  return true;
}

/* Handle SD write errors */
static bool sd_handleWriteError() {
  sdStatus.writeErrors++;

  if (++sdWriteErrorCount >= 5) {
    Serial.println("[SD] Too many write errors - marking not ready");
    sdStatus.isReady = false;
    return false;
  }
  return true;
}

/* Initialize SD card */
bool sd_init() {
  if (sdStatus.isInitialized) {
    Serial.println("[SD] Already initialized");
    return true;
  }

  if (!sd_ringBuffer_init()) return false;

  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[SD] Failed to init SD_MMC");
    sdStatus.isInitialized = false;
    sdStatus.isReady = false;
    return false;
  }

  Serial.println("[SD] SD_MMC initialized");

  uint64_t cardSize = SD_MMC.cardSize();
  Serial.printf("[SD] Card: %llu MB\n", cardSize / (1024 * 1024));

  if (!sd_createNewFile()) {
    Serial.println("[SD] Failed to create initial file");
    sdStatus.isInitialized = false;
    sdStatus.isReady = false;
    return false;
  }

  sdStatus.isInitialized = true;
  sdStatus.isReady = true;
  sdStatus.packetsWritten = 0;
  sdStatus.writeErrors = 0;
  sdWriteErrorCount = 0;

  Serial.println("[SD] Init complete");
  return true;
}

/* Check if SD is ready */
bool sd_isReady() {
  return sdStatus.isReady && sdStatus.isInitialized && (sdLogFile != NULL);
}

/* Write packet to SD */
bool sd_writePacket(const FeaturePacket* packet) {
  if (!packet || !sd_isReady()) return false;

  if (fwrite(packet, sizeof(FeaturePacket), 1, sdLogFile) != 1) {
    return sd_handleWriteError();
  }

  sdWriteErrorCount = 0;
  sdStatus.packetsWritten++;
  sdPacketsInCurrentFile++;

  if (sdPacketsInCurrentFile >= MAX_PACKETS_PER_FILE) {
    return sd_rotateFile();
  }

  return true;
}

/* Rotate to new file */
bool sd_rotateFile() {
  if (sdLogFile) {
    fflush(sdLogFile);
    fclose(sdLogFile);
    sdLogFile = NULL;
    Serial.printf("[SD] Closed: %u packets\n", sdPacketsInCurrentFile);
  }

  if (!sd_createNewFile()) {
    Serial.println("[SD] Failed to create rotated file");
    sdStatus.isReady = false;
    return false;
  }

  return true;
}

/* Flush current file */
void sd_flush() {
  if (sdLogFile) {
    fflush(sdLogFile);
    Serial.printf("[SD] Flushed: %u packets\n", sdPacketsInCurrentFile);
  }
}

/* Close current file */
void sd_closeFile() {
  if (sdLogFile) {
    fflush(sdLogFile);
    fclose(sdLogFile);
    sdLogFile = NULL;
    Serial.printf("[SD] Closed: %u packets total\n", sdPacketsInCurrentFile);
    sdPacketsInCurrentFile = 0;
  }
}

/* Get SD status */
SDCardStatus sd_getStatus() {
  sdStatus.bufferStats = sdRingBuffer.stats;
  return sdStatus;
}

/* Get available space */
uint64_t sd_getAvailableSpace() {
  if (!sdStatus.isInitialized) return 0;

  uint64_t cardSize = SD_MMC.cardSize();
  uint64_t used = sdStatus.packetsWritten * sizeof(FeaturePacket);

  return (used >= cardSize) ? 0 : cardSize - used;
}

/* Get packet count */
uint32_t sd_getPacketCount() {
  return sdStatus.packetsWritten;
}

/* Print diagnostics */
void sd_printStatus() {
  SDCardStatus status = sd_getStatus();

  Serial.println("\n========== SD Card Status ==========");
  Serial.printf("Initialized:      %s\n", status.isInitialized ? "Yes" : "No");
  Serial.printf("Ready:            %s\n", status.isReady ? "Yes" : "No");
  Serial.printf("Packets Written:  %u\n", status.packetsWritten);
  Serial.printf("Files Created:    %u\n", status.filesCreated);
  Serial.printf("Write Errors:     %u\n", status.writeErrors);
  Serial.printf("Current File:     %s\n", status.currentFilePath);

  uint64_t avail = sd_getAvailableSpace();
  Serial.printf("Available Space:  %llu MB\n", avail / (1024 * 1024));

  Serial.println("\n====== Ring Buffer Status ======");
  Serial.printf("Current Depth:    %u / %u packets\n",
    sdRingBuffer.size, sdRingBuffer.capacity);
  Serial.printf("Total Enqueued:   %u\n", status.bufferStats.totalEnqueued);
  Serial.printf("Total Dequeued:   %u\n", status.bufferStats.totalDequeued);
  Serial.printf("Peak Depth:       %u packets\n", status.bufferStats.peakQueueDepth);
  Serial.printf("Dropped Packets:  %u\n", status.bufferStats.droppedPackets);
  Serial.printf("Mutex Contentions:%u\n", status.bufferStats.mutexContentions);

  float util = (float)sdRingBuffer.size * 100.0f / sdRingBuffer.capacity;
  Serial.printf("Utilization:      %.1f%%\n", util);

  const char* policy[] = {"DROP_OLDEST", "DROP_NEWEST"};
  Serial.printf("Backpressure:     %s\n", policy[sdRingBuffer.policy]);

  Serial.println("================================\n");
}
