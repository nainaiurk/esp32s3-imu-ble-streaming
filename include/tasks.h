#ifndef TASKS_H
#define TASKS_H

#include "data_types.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Global data packets
extern ImuPacket imuPacket;
extern FeaturePacket featurePacket;

// Mutexes for thread-safe access
extern SemaphoreHandle_t imuDataMutex;
extern SemaphoreHandle_t featureDataMutex;

// FreeRTOS task functions
void imuSamplingTask(void* parameter);
void featureComputationTask(void* parameter);
void bleTask(void* parameter);

// Initialize all tasks
void initTasks();

#endif
