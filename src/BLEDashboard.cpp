#include "BLEDashboard.h"
#include <NimBLEDevice.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_TX_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_RX_UUID "828917c1-ea55-4d4a-a66e-fd202cea0645"
#define CHAR_LOG_UUID "5111b1db-2917-48f8-b3d5-e51c8f35fc06"

NimBLEServer *pServer = NULL;
NimBLECharacteristic *pTxCharacteristic = NULL;
NimBLECharacteristic *pRxCharacteristic = NULL;
NimBLECharacteristic *pLogCharacteristic = NULL;

bool deviceConnected = false;
bool restartAdvertising = false;
char log_buffer[1024] = "System Booting...\n";
uint16_t phone_conn_id = 0xFFFF;

void addLog(const char *msg)
{
  Serial.println(msg);
  int msg_len = strlen(msg);
  if (msg_len > 256)
    return;
  int cur_len = strlen(log_buffer);

  if (cur_len + msg_len + 2 > sizeof(log_buffer))
  {
    int shift = (cur_len + msg_len + 2) - sizeof(log_buffer);
    if (shift >= cur_len)
    {
      log_buffer[0] = '\0';
      cur_len = 0;
    }
    else
    {
      memmove(log_buffer, log_buffer + shift, cur_len - shift + 1);
      cur_len -= shift;
    }
  }
  strcat(log_buffer, msg);
  strcat(log_buffer, "\n");

  if (deviceConnected && pLogCharacteristic != NULL)
  {
    pLogCharacteristic->setValue((uint8_t *)msg, strlen(msg));
    pLogCharacteristic->notify();
  }
}

class MyServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc)
  {
    pServer->updateConnParams(desc->conn_handle, 12, 12, 0, 60);
    phone_conn_id = desc->conn_handle;
    deviceConnected = true;
    addLog("[BLE] Dashboard Connected!");
  }
  void onDisconnect(NimBLEServer *pServer, ble_gap_conn_desc *desc)
  {
    restartAdvertising = true;
    if (desc->conn_handle == phone_conn_id)
    {
      deviceConnected = false;
      phone_conn_id = 0xFFFF;
      addLog("[BLE] Dashboard Disconnected!");
    }
  }
};

class MyRxCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pCharacteristic)
  {
    String rxValue = pCharacteristic->getValue().c_str();
    if (rxValue.length() > 0)
    {
      if (rxValue == "OTA")
        triggerOTA();
      else if (rxValue == "SCAN")
        triggerScan();
      else if (rxValue == "SAVE")
        triggerEEPROMSave();
      else if (rxValue == "GET")
      {
        char msg[200];
        snprintf(msg, sizeof(msg), "{\"cfg\":1,\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,\"ms\":%.2f,\"tc\":%.2f,\"s\":\"%s\"}",
                 deviceInfo.vel_Kp, deviceInfo.vel_Ki, deviceInfo.vel_Kd,
                 deviceInfo.max_speed, deviceInfo.brakeTimeConstant, deviceInfo.home_ssid);
        pTxCharacteristic->setValue((uint8_t *)msg, strlen(msg));
        pTxCharacteristic->notify();
      }
      else if (rxValue.startsWith("CFG:"))
      {
        // Parse CFG:kp:ki:kd:ms:tc
        int s1 = rxValue.indexOf(':', 4);
        int s2 = rxValue.indexOf(':', s1 + 1);
        int s3 = rxValue.indexOf(':', s2 + 1);
        int s4 = rxValue.indexOf(':', s3 + 1);

        if (s1 > 0 && s2 > 0 && s3 > 0 && s4 > 0)
        {
          deviceInfo.vel_Kp = rxValue.substring(4, s1).toFloat();
          deviceInfo.vel_Ki = rxValue.substring(s1 + 1, s2).toFloat();
          deviceInfo.vel_Kd = rxValue.substring(s2 + 1, s3).toFloat();
          deviceInfo.max_speed = rxValue.substring(s3 + 1, s4).toFloat();
          deviceInfo.brakeTimeConstant = rxValue.substring(s4 + 1).toFloat();
          addLog("Live Tuning Updated.");
        }
      }
      else if (rxValue.startsWith("WIFI:"))
      {
        int s1 = rxValue.indexOf(':', 5);
        if (s1 > 0)
          triggerWiFiSave(rxValue.substring(5, s1), rxValue.substring(s1 + 1));
      }
    }
  }
};

void dash_begin()
{
  NimBLEDevice::init("E-Bike Pusher");
  NimBLEDevice::setMTU(256);
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  NimBLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHAR_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  pRxCharacteristic = pService->createCharacteristic(CHAR_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pRxCharacteristic->setCallbacks(new MyRxCallbacks());
  pLogCharacteristic = pService->createCharacteristic(CHAR_LOG_UUID, NIMBLE_PROPERTY::NOTIFY);
  pService->start();
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
}

void dash_loop()
{
  if (restartAdvertising)
  {
    delay(200);
    NimBLEDevice::getAdvertising()->start();
    restartAdvertising = false;
  }
}

void dash_sendTelemetry(int cadence, float power, float voltage, float current, float brake_avg, float target_val, float actual_vel, int mode)
{
  if (deviceConnected)
  {
    char json[150];
    snprintf(json, sizeof(json), "{\"c\":%d,\"p\":%.1f,\"v\":%.1f,\"a\":%.1f,\"pr\":%.2f,\"tq\":%.2f,\"av\":%.2f,\"m\":%d}",
             cadence, power, voltage, current, brake_avg, target_val, actual_vel, mode);
    pTxCharacteristic->setValue((uint8_t *)json, strlen(json));
    pTxCharacteristic->notify();
  }
}