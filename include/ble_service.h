#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <NimBLEDevice.h>

void initBLEService();
bool isBLEConnected();


NimBLECharacteristic* getBLECharacteristic();

#endif
