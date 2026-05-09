#ifndef BLE_DASHBOARD_H
#define BLE_DASHBOARD_H

#include <Arduino.h>

struct DeviceInfo
{
  char macAddress[18];
  char deviceName[20];
  uint8_t addressType;
  bool SCAN_FOR_DEVICE;
  float torqueMultiplier;
  float brakeTimeConstant;
  char home_ssid[32];
  char home_pass[64];
  bool maintenanceMode;
};
extern DeviceInfo deviceInfo;

// THE NEW BRIDGE: A clean struct so main.cpp doesn't have to worry about JSON formatting
struct TelemetryData
{
  int cadence;
  float mech_power;
  float vbus;
  float phase_current;
  float brake_filter;
  float target_val;
  int odrive_mode;
};

void addLog(const char *msg);
void dash_begin();
void dash_loop();
void dash_sendTelemetry(TelemetryData &data); // Pass the struct instead of 7 variables!

extern void triggerEEPROMSave();
extern void triggerOTA();
extern void triggerScan();
extern void triggerWiFiSave(String ssid, String pass);

#endif