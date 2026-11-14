#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Hardware UART1 to STM32 (RX: GPIO20, TX: GPIO21)
HardwareSerial STM32_Serial(1);

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      BLEDevice::startAdvertising(); // Allow reconnecting immediately
    }
};

class RxCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        // Forward received bytes directly to STM32 over hardware UART
        for (size_t i = 0; i < rxValue.length(); i++) {
          STM32_Serial.write(rxValue[i]);
        }
      }
    }
};

void setup() {
  // Initialize UART to STM32 on Pins 20 and 21 at 115200 baud
  STM32_Serial.begin(115200, SERIAL_8N1, 20, 21);

  BLEDevice::init("RobotArm_C3");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // TX Characteristic (Feedback to PC)
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX Characteristic (Writable from PC)
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_WRITE_NR
                                         );
  pRxCharacteristic->setCallbacks(new RxCallbacks());
  pService->start();

  // Split advertising packets to stay within 31-byte legacy limit
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  
  BLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setCompleteServices(BLEUUID(SERVICE_UUID));
  pAdvertising->setAdvertisementData(advData);

  BLEAdvertisementData scanData;
  scanData.setName("RobotArm_C3");
  pAdvertising->setScanResponseData(scanData);

  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void loop() {
  // Echo any feedback received from STM32 back to PC over BLE
  if (deviceConnected && STM32_Serial.available()) {
    String feedback = "";
    while (STM32_Serial.available()) {
      feedback += (char)STM32_Serial.read();
    }
    pTxCharacteristic->setValue(feedback.c_str());
    pTxCharacteristic->notify();
  }
}