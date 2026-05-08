#ifndef BLE_DASHBOARD_H
#define BLE_DASHBOARD_H

#include <Arduino.h>
#include <esp_bt_defs.h>

// Global Struct so all files can read it
struct DeviceInfo {
  char macAddress[18];
  char deviceName[20];
  esp_ble_addr_type_t addressType;
  bool SCAN_FOR_DEVICE;
  int brakingThreshold;
  int brakingTimeout;
  float torqueMultiplier;
  float brakeAlpha;
  char home_ssid[32];
  char home_pass[64];
  bool maintenanceMode;
};
extern DeviceInfo deviceInfo;

// Global Log Function
void addLog(const char* msg);

// Dashboard Module
void dash_begin();
void dash_loop();
void dash_sendTelemetry(int cadence, float power, float voltage, float current, float brake_avg, bool isBraking);

// Hardware Triggers for main.cpp
extern void triggerEEPROMSave();
extern void triggerOTA();
extern void triggerScan();
extern void triggerWiFiSave(String ssid, String pass);

#endif