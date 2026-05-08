#include "BLEDashboard.h"
#include <NimBLEDevice.h> // <--- NEW LIBRARY

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_TX_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_RX_UUID        "828917c1-ea55-4d4a-a66e-fd202cea0645"
#define CHAR_LOG_UUID       "5111b1db-2917-48f8-b3d5-e51c8f35fc06"

NimBLEServer* pServer = NULL;
NimBLECharacteristic* pTxCharacteristic = NULL;
NimBLECharacteristic* pLogCharacteristic = NULL;

bool deviceConnected = false;
bool restartAdvertising = false;
char log_buffer[1024] = "System Booting...\n";
uint16_t phone_conn_id = 0xFFFF;

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
  strcat(log_buffer, msg); strcat(log_buffer, "\n");

  if (deviceConnected && pLogCharacteristic != NULL) {
    pLogCharacteristic->setValue((uint8_t*)msg, strlen(msg));
    pLogCharacteristic->notify();
  }
}

class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) { 
        pServer->startAdvertising(); 
        phone_conn_id = desc->conn_handle; 
        deviceConnected = true; 
        Serial.println("\n[BLE] Phone Dashboard Connected!");
    }
    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) { 
        restartAdvertising = true; 
        if (desc->conn_handle == phone_conn_id) {
            deviceConnected = false; 
            phone_conn_id = 0xFFFF;
            Serial.println("\n[BLE] Phone Dashboard Disconnected!");
        }
    }
};

class MyRxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue().c_str();
        if (rxValue.length() > 0) {
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
  NimBLEDevice::init("E-Bike Pusher");
  
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  NimBLEService *pService = pServer->createService(SERVICE_UUID);
  
  // NimBLE auto-creates 2902 descriptors!
  pTxCharacteristic = pService->createCharacteristic(CHAR_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  
  NimBLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHAR_RX_UUID, NIMBLE_PROPERTY::WRITE);
  pRxCharacteristic->setCallbacks(new MyRxCallbacks());

  pLogCharacteristic = pService->createCharacteristic(CHAR_LOG_UUID, NIMBLE_PROPERTY::NOTIFY);
  
  pService->start();
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
}

void dash_loop() {
  if (restartAdvertising) {
    delay(200);
    pServer->startAdvertising();
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