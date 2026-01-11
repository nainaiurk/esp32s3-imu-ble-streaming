#include "sd_card_ring_buffer.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
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

// -------------- Ring Buffer Operations ---------------

bool sd_ringBuffer_init() {
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

//------------Enqueue packet - DROP_OLDEST when full-----------
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

// ------------Dequeue oldest packet--------------
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

// -----------Clear all packets from ring buffer---------------
void sd_clearBuffer() {
  if (!sdRingBufferMutex) return;

  if (xSemaphoreTake(sdRingBufferMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

  sdRingHead = 0;
  sdRingTail = 0;
  sdRingSize = 0;

  xSemaphoreGive(sdRingBufferMutex);
}

//  ------------------Get current number of packets in buffer------------
uint32_t sd_getPacketCount() {
  if (!sdRingBufferMutex) return 0;

  if (xSemaphoreTake(sdRingBufferMutex, 0) != pdTRUE) return 0;

  uint32_t count = sdRingSize;
  xSemaphoreGive(sdRingBufferMutex);
  return count;
}

// --------------Get ring buffer statistics ----------------
SDRingBufferStats sd_getRingBufferStats() {
  SDRingBufferStats stats = {0, 0, 0};

  if (!sdRingBufferMutex) return stats;

  if (xSemaphoreTake(sdRingBufferMutex, pdMS_TO_TICKS(100)) != pdTRUE) return stats;

  stats.totalEnqueued = sdRingTotalEnqueued;
  stats.totalDequeued = sdRingTotalDequeued;
  stats.droppedPackets = sdRingDroppedPackets;

  xSemaphoreGive(sdRingBufferMutex);
  return stats;
}
