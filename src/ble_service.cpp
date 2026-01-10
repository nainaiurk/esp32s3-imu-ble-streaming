#include "ble_service.h"
#include "config.h"
#include "data_types.h"
#include <Arduino.h>
#include <freertos/event_groups.h>

// Event group bit definitions
#define BLE_CONNECTED_BIT    (1 << 0)

extern EventGroupHandle_t taskEventGroup;

static NimBLECharacteristic* pCharacteristic = nullptr;
static volatile bool deviceConnected = false;

// Server callback class
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) override {
    deviceConnected = true;
    xEventGroupSetBits(taskEventGroup, BLE_CONNECTED_BIT);
    NimBLEDevice::stopAdvertising();

    auto peer = pServer->getPeerInfo(0);
    pServer->updateConnParams(
      peer.getConnHandle(),
      12, 12, 0, 60
    );
  }
  
  void onDisconnect(NimBLEServer* pServer) override {
    deviceConnected = false;
    xEventGroupClearBits(taskEventGroup, BLE_CONNECTED_BIT);
    NimBLEDevice::startAdvertising();
  }
};

void initBLEService() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setMTU(247);

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  pService->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();
}

bool isBLEConnected() {
  return deviceConnected;
}

NimBLECharacteristic* getBLECharacteristic() {
  return pCharacteristic;
}
