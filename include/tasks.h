#ifndef TASKS_H
#define TASKS_H

#include "data_types.h"
#include "sd_card_data.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

// Event Group bits for task synchronization
#define BLE_CONNECTED_BIT    (1 << 0)  // BLE device connected
#define SD_READY_BIT         (1 << 1)  // SD card ready
#define POWER_SAVE_MODE_BIT  (1 << 2)  // Power-saving mode active

// Global data packets
extern ImuPacket imuPacket;
extern volatile bool isLowPowerMode;  // Power-saving mode flag
extern FeaturePacket featurePacket;

// Mutexes for thread-safe access
extern SemaphoreHandle_t imuDataMutex;
extern SemaphoreHandle_t featureDataMutex;

// Event group for inter-task synchronization
extern EventGroupHandle_t taskEventGroup;

// FreeRTOS task functions
void imuSamplingTask(void* parameter);
void featureComputationTask(void* parameter);
void bleTask(void* parameter);
void sdLoggingTask(void* parameter);
void imuDebugTask(void* parameter);

// Initialize all tasks and event groups
void initTasks();

#endif
