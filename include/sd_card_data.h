#ifndef SD_CARD_DATA_H
#define SD_CARD_DATA_H

#include "data_types.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// SD card status structure
typedef struct {
  bool isInitialized;
  bool isReady;
  uint32_t packetsWritten;
  uint32_t filesCreated;
  uint32_t writeErrors;
  char currentFilePath[128];
} SDCardStatus;

bool sd_init();
bool sd_writePacket(const FeaturePacket* packet);
bool sd_rotateFile();
void sd_closeFile();

SDCardStatus sd_getStatus();
uint64_t sd_getAvailableSpace();
void sd_getFileName(char* buffer, size_t bufferSize);
bool sd_isReady();
void sd_flush();
uint32_t sd_getPacketCount();

#endif
