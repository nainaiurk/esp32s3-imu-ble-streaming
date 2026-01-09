#include "ble_service.h"
#include "config.h"
#include <Arduino.h>

static NimBLECharacteristic* pCharacteristic = nullptr;
static bool deviceConnected = false;

// Server callback class
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) override {
    deviceConnected = true;
    NimBLEDevice::stopAdvertising();

    auto peer = pServer->getPeerInfo(0);
    pServer->updateConnParams(
      peer.getConnHandle(),
      12, 12, 0, 60
    );

    Serial.println("Client connected");
  }
  
  void onDisconnect(NimBLEServer* pServer) override {
    deviceConnected = false;
    NimBLEDevice::startAdvertising();
    Serial.println("Client disconnected");
  }
  
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) override {
    Serial.printf("MTU updated: %u\n", MTU);
  }
};

// Characteristic callback class
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic) override {
    std::string rx = pCharacteristic->getValue();
    Serial.print("Received: ");
    Serial.println(rx.c_str());
  }
};

void initBLEService() {
  Serial.println("Starting NimBLE Server...");

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setMTU(247);

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  pCharacteristic->setCallbacks(new CharacteristicCallbacks());
  pCharacteristic->setValue("Hello from ESP32!");

  pService->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("BLE Server ready");
}

bool isBLEConnected() {
  return deviceConnected;
}

NimBLECharacteristic* getBLECharacteristic() {
  return pCharacteristic;
}
