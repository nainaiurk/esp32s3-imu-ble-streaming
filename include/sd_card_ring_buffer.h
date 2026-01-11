#ifndef SD_CARD_RING_BUFFER_H
#define SD_CARD_RING_BUFFER_H

#include "data_types.h"
#include <stdint.h>
#include <stdbool.h>

// Ring buffer stats - production essentials only
typedef struct {
  uint32_t totalEnqueued;      // Total packets queued
  uint32_t totalDequeued;      // Total packets written
  uint32_t droppedPackets;     // Packets dropped (buffer full)
} SDRingBufferStats;

// Ring buffer initialization
bool sd_ringBuffer_init();

// Ring buffer operations (DROP_OLDEST policy: oldest packet dropped when full)
bool sd_enqueue(const FeaturePacket* packet);    // Queue packet
bool sd_dequeue(FeaturePacket* packet);          // Dequeue oldest packet
void sd_clearBuffer();                           // Clear all buffered packets
uint32_t sd_getPacketCount();                    // Get current number of packets in buffer

// Ring buffer statistics
SDRingBufferStats sd_getRingBufferStats();

#endif
