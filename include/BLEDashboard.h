#ifndef BLE_DASHBOARD_H
#define BLE_DASHBOARD_H

#include <Arduino.h>

struct DeviceInfo
{
  char macAddress[18];
  char deviceName[20];
  uint8_t addressType;
  bool SCAN_FOR_DEVICE;
  float vel_Kp;    // NEW: Proportional Gain (Punch)
  float vel_Ki;    // NEW: Integral Gain (Acceleration)
  float max_speed; // NEW: Speed limit (rev/s)
  float brakeTimeConstant;
  char home_ssid[32];
  char home_pass[64];
  bool maintenanceMode;
};
extern DeviceInfo deviceInfo;

void addLog(const char *msg);

void dash_begin();
void dash_loop();
void dash_sendTelemetry(int cadence, float power, float vbus, float current, float brake_avg, float target_val, float actual_vel, int mode);

extern void triggerEEPROMSave();
extern void triggerOTA();
extern void triggerScan();
extern void triggerWiFiSave(String ssid, String pass);

#endif