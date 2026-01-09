#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <stdint.h>

void initMPU6050();
bool readIMUData(int16_t rawAccel[3], int16_t rawGyro[3], int16_t& rawTemp);

#endif
