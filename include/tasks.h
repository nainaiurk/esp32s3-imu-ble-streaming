#ifndef TASKS_H
#define TASKS_H

#include "data_types.h"
#include "sd_card_data.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

// Event Group bits for task synchronization
#define FEATURE_READY_BIT    (1 << 0)  // Feature computation completed
#define BLE_CONNECTED_BIT    (1 << 1)  // BLE device connected
#define SD_READY_BIT         (1 << 2)  // SD card ready
#define BUFFER_FULL_BIT      (1 << 3)  // SD write buffer is full
#define SHUTDOWN_BIT         (1 << 4)  // Graceful shutdown signal

// Queue sizes
#define SD_LOG_QUEUE_SIZE    256       // Max buffered packets for SD logging

// Global data packets
extern ImuPacket imuPacket;
extern FeaturePacket featurePacket;

// Mutexes for thread-safe access
extern SemaphoreHandle_t imuDataMutex;
extern SemaphoreHandle_t featureDataMutex;

// Event group for inter-task synchronization
extern EventGroupHandle_t taskEventGroup;

// Queue for SD logging
extern QueueHandle_t sdLogQueue;

// FreeRTOS task functions
void imuSamplingTask(void* parameter);           // Task 1: IMU sampling (50 Hz)
void featureComputationTask(void* parameter);    // Task 2: Feature processing (50 Hz)
void bleTask(void* parameter);                   // Task 3: BLE notifications (10 Hz)
void sdLoggingTask(void* parameter);             // Task 4: SD card buffered writes

// Initialize all tasks, queues, and event groups
void initTasks();

// Utility function to send feature packet to SD logging queue
void queueFeatureForSD(const FeaturePacket* packet);

#endif
