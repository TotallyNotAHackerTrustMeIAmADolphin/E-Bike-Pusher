#include "BLEDashboard.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_TX_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_RX_UUID        "828917c1-ea55-4d4a-a66e-fd202cea0645"
#define CHAR_LOG_UUID       "5111b1db-2917-48f8-b3d5-e51c8f35fc06"

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pLogCharacteristic = NULL;

bool deviceConnected = false;
bool restartAdvertising = false;
char log_buffer[1024] = "System Booting...\n";

// NEW: Safely add to memory, but DO NOT transmit here to prevent threading crashes!
void addLog(const char* msg) {
  Serial.println(msg);
  int msg_len = strlen(msg);
  if (msg_len > 256) return; 
  int cur_len = strlen(log_buffer);
  
  if (cur_len + msg_len + 2 > sizeof(log_buffer)) {
    int shift = (cur_len + msg_len + 2) - sizeof(log_buffer);
    if (shift >= cur_len) { log_buffer[0] = '\0'; cur_len = 0; } 
    else { memmove(log_buffer, log_buffer + shift, cur_len - shift + 1); cur_len -= shift; }
  }
  strcat(log_buffer, msg); 
  strcat(log_buffer, "\n");
}

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { 
      deviceConnected = true; 
      Serial.println("[BLE] Phone Connected!");
    }
    void onDisconnect(BLEServer* pServer) { 
        deviceConnected = false; 
        restartAdvertising = true; 
        Serial.println("[BLE] Phone Disconnected!");
    }
};

class MyRxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue().c_str();
        if (rxValue.length() > 0) {
            Serial.print("BLE RX: "); Serial.println(rxValue);
            
            if (rxValue == "OTA") triggerOTA();
            else if (rxValue == "SCAN") triggerScan();
            else if (rxValue == "SAVE") triggerEEPROMSave();
            else if (rxValue == "GET") {
                char msg[200];
                snprintf(msg, sizeof(msg), "{\"cfg\":1,\"th\":%d,\"ti\":%d,\"tm\":%.3f,\"ba\":%.2f,\"s\":\"%s\",\"p\":\"%s\"}",
                         deviceInfo.brakingThreshold, deviceInfo.brakingTimeout, deviceInfo.torqueMultiplier, 
                         deviceInfo.brakeAlpha, deviceInfo.home_ssid, deviceInfo.home_pass);
                pTxCharacteristic->setValue((uint8_t*)msg, strlen(msg));
                pTxCharacteristic->notify();
            }
            else if (rxValue.startsWith("CFG:")) {
                int s1 = rxValue.indexOf(':', 4), s2 = rxValue.indexOf(':', s1 + 1), s3 = rxValue.indexOf(':', s2 + 1);
                if(s1 > 0 && s2 > 0 && s3 > 0) {
                    deviceInfo.brakingThreshold = rxValue.substring(4, s1).toInt();
                    deviceInfo.brakingTimeout = rxValue.substring(s1 + 1, s2).toInt();
                    deviceInfo.torqueMultiplier = rxValue.substring(s2 + 1, s3).toFloat();
                    deviceInfo.brakeAlpha = rxValue.substring(s3 + 1).toFloat();
                    Serial.println("RAM settings updated!");
                }
            }
            else if (rxValue.startsWith("WIFI:")) {
                int s1 = rxValue.indexOf(':'), s2 = rxValue.indexOf(':', s1 + 1);
                if(s1 > 0 && s2 > 0) triggerWiFiSave(rxValue.substring(s1 + 1, s2), rxValue.substring(s2 + 1));
            }
        }
    }
};

void dash_begin() {
  BLEDevice::init("E-Bike Pusher");
  BLEDevice::setMTU(256); // A safer MTU size for all phones

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pTxCharacteristic = pService->createCharacteristic(CHAR_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHAR_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyRxCallbacks());

  pLogCharacteristic = pService->createCharacteristic(CHAR_LOG_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pLogCharacteristic->addDescriptor(new BLE2902());
  
  pService->start();
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
}

void dash_loop() {
  // Safely restart advertising in the main loop thread!
  if (restartAdvertising) {
    delay(200);
    pServer->startAdvertising();
    Serial.println("[BLE] Advertising Restarted.");
    restartAdvertising = false;
  }
}

void dash_sendTelemetry(int cadence, float power, float voltage, float current, float brake_avg, bool isBraking) {
  if (deviceConnected) {
    char json[120];
    snprintf(json, sizeof(json), "{\"c\":%d,\"p\":%.1f,\"v\":%.1f,\"a\":%.1f,\"pr\":%.2f,\"b\":%d}", 
             cadence, power, voltage, current, brake_avg, isBraking?1:0);
    pTxCharacteristic->setValue((uint8_t*)json, strlen(json));
    pTxCharacteristic->notify();
  }
}