#ifndef SD_CARD_DATA_H
#define SD_CARD_DATA_H

#include "data_types.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Backpressure policies for ring buffer
typedef enum {
  SD_BACKPRESSURE_DROP_OLDEST, // Drop oldest packet when full (explicit loss)
  SD_BACKPRESSURE_DROP_NEWEST  // Drop newest packet when full (reject)
} SDBackpressurePolicy;

// Ring buffer stats
typedef struct {
  uint32_t totalEnqueued;      // Total packets queued
  uint32_t totalDequeued;      // Total packets written
  uint32_t peakQueueDepth;     // Maximum queue depth reached
  uint32_t droppedPackets;     // Packets dropped due to backpressure
} SDRingBufferStats;

// SD card status structure
typedef struct {
  bool isInitialized;
  bool isReady;
  uint32_t packetsWritten;
  uint32_t filesCreated;
  uint32_t writeErrors;
  char currentFilePath[128];
  SDRingBufferStats bufferStats;  // Ring buffer statistics
} SDCardStatus;

// Ring buffer queue operations
bool sd_enqueue(const FeaturePacket* packet);           // Queue packet with backpressure
bool sd_dequeue(FeaturePacket* packet);                // Dequeue oldest packet
uint32_t sd_getRingBufferSize();                       // Current queue depth
uint32_t sd_getRingBufferCapacity();                   // Max queue capacity
void sd_setBackpressurePolicy(SDBackpressurePolicy p); // Configure backpressure behavior

// Core SD operations
bool sd_init();
bool sd_writePacket(const FeaturePacket* packet);
bool sd_rotateFile();
void sd_closeFile();

// Status & monitoring
SDCardStatus sd_getStatus();
uint64_t sd_getAvailableSpace();
void sd_getFileName(char* buffer, size_t bufferSize);
bool sd_isReady();
void sd_flush();
uint32_t sd_getPacketCount();
void sd_printStatus();

#endif
