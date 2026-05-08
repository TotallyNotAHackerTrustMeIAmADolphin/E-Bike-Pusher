#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include "ODriveCAN.h"
#include "CadenceSensor.h"
#include "BLEDashboard.h"

ODriveCAN odrive(0);
CadenceSensor cadenceSensor;
DeviceInfo deviceInfo; 

#define EEPROM_SIZE 256
constexpr int EEPROM_ADDRESS = 0;

#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_16
#define inductiveProbe 34 
#define pullupPowerPin 33 

float brake_avg = 1.0f; 
float target_motor_torque = 0.0f; 

void triggerEEPROMSave() {
  EEPROM.put(EEPROM_ADDRESS, deviceInfo); EEPROM.commit();
  addLog("Tune permanently saved to EEPROM!");
}
void triggerOTA() {
  deviceInfo.maintenanceMode = true; triggerEEPROMSave();
  addLog("Rebooting to OTA Maintenance Mode...");
  delay(500); ESP.restart();
}
void triggerScan() {
  deviceInfo.SCAN_FOR_DEVICE = true; triggerEEPROMSave();
  addLog("Rebooting to Scan for Cadence Sensor...");
  delay(500); ESP.restart();
}
void triggerWiFiSave(String s, String p) {
  strlcpy(deviceInfo.home_ssid, s.c_str(), sizeof(deviceInfo.home_ssid));
  strlcpy(deviceInfo.home_pass, p.c_str(), sizeof(deviceInfo.home_pass));
  triggerEEPROMSave();
  addLog("WiFi Saved! Rebooting...");
  delay(500); ESP.restart();
}

void runMaintenanceMode() {
  addLog("MAINTENANCE MODE: Turning on WiFi for OTA.");
  deviceInfo.maintenanceMode = false; triggerEEPROMSave();

  WiFi.mode(WIFI_STA);
  WiFi.begin(deviceInfo.home_ssid, deviceInfo.home_pass);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); attempts++; }

  if (WiFi.status() == WL_CONNECTED) {
    char m[64]; snprintf(m, sizeof(m), "Connected! IP: %s", WiFi.localIP().toString().c_str()); addLog(m);
  } else {
    WiFi.mode(WIFI_AP); WiFi.softAP("ESP-Maintenance", "12345678");
    addLog("WiFi failed. Started AP: ESP-Maintenance");
  }

  ArduinoOTA.setHostname("odrive-node");
  ArduinoOTA.begin();
  
  while (true) { ArduinoOTA.handle(); delay(2); }
}

void setup() {
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRESS, deviceInfo);
  
  if(deviceInfo.brakingThreshold <= 0 || deviceInfo.brakingThreshold > 4096 || isnan(deviceInfo.torqueMultiplier)) {
      deviceInfo.brakingThreshold = 2048; 
      deviceInfo.torqueMultiplier = 5.0; 
      deviceInfo.brakeTimeConstant = 1.0;     
      strncpy(deviceInfo.home_ssid, "wlesswg", 31);
      strncpy(deviceInfo.home_pass, "hba.1245", 63);
      deviceInfo.maintenanceMode = false;
      EEPROM.put(EEPROM_ADDRESS, deviceInfo); EEPROM.commit();
      addLog("EEPROM format shifted. Restored safe defaults!");
  }

  if (deviceInfo.maintenanceMode) runMaintenanceMode(); 

  pinMode(pullupPowerPin, OUTPUT); digitalWrite(pullupPowerPin, HIGH); 
  pinMode(inductiveProbe, INPUT); 

  dash_begin();
  
  odrive.begin(CAN_TX_PIN, CAN_RX_PIN);
  delay(250);
  odrive.setMode(1, 1); 
  delay(50);
  odrive.setTorque(0.0); 
  odrive.setVelocity(0.0); 
  delay(10);
  odrive.setState(8); 

  cadenceSensor.begin(deviceInfo.SCAN_FOR_DEVICE, deviceInfo.macAddress, deviceInfo.addressType);
  addLog("System Boot Sequence Complete.");
}

unsigned long last_cmd_time = 0;
unsigned long last_dashboard_time = 0;

void loop() {
  odrive.poll();
  cadenceSensor.loop();
  dash_loop();

  if (cadenceSensor.foundNewDevice()) {
    strlcpy(deviceInfo.macAddress, cadenceSensor.getNewMac(), sizeof(deviceInfo.macAddress));
    strlcpy(deviceInfo.deviceName, cadenceSensor.getNewName(), sizeof(deviceInfo.deviceName));
    deviceInfo.addressType = cadenceSensor.getNewAddressType();
    deviceInfo.SCAN_FOR_DEVICE = false; 
    triggerEEPROMSave();
    cadenceSensor.clearNewDeviceFlag();
    addLog("Cadence Sensor paired & saved!");
  }

  // --- MOTOR CONTROL (50Hz = 0.02s) ---
  if (millis() - last_cmd_time >= 20) {
    last_cmd_time = millis();
    
    int rawProbe = analogRead(inductiveProbe);
    float target_state = (rawProbe < deviceInfo.brakingThreshold) ? -1.0f : 1.0f;
    
    // Mathematical Time Constant Smoothing (Low Pass Filter)
    float dt = 0.02f; // Loop time (20ms)
    float tau = deviceInfo.brakeTimeConstant;
    if (tau < 0.01f) tau = 0.01f; 
    float alpha = dt / (tau + dt);
    
    brake_avg = (alpha * target_state) + ((1.0f - alpha) * brake_avg);

    // The New Physics Logic!
    if (brake_avg > 0.0f) {
        if (cadenceSensor.getCadence() > 0) {
            target_motor_torque = deviceInfo.torqueMultiplier * brake_avg;
        } else {
            target_motor_torque = 0.0f; 
        }
    } else {
        // Apply Regen Braking proportionally! (No pedaling required to brake)
        target_motor_torque = deviceInfo.torqueMultiplier * brake_avg;
    }

    odrive.setTorque(target_motor_torque);
    odrive.requestData(CMD_GET_ENCODER_ESTIMATES);
    odrive.requestData(CMD_GET_IQC);
    odrive.requestData(CMD_GET_VBUS_VOLTAGE); 
  }

  // --- BLUETOOTH DASHBOARD NOTIFY (2Hz) ---
  if (millis() - last_dashboard_time >= 500) {
    last_dashboard_time = millis();
    float mech_power = abs((odrive.getCurrent() * 0.356) * (odrive.getVelocity() * 6.283185));
    bool isBraking = (brake_avg < 0.0f); // Dashboard turns red if doing active braking
    
    dash_sendTelemetry(cadenceSensor.getCadence(), mech_power, odrive.getVoltage(), odrive.getCurrent(), brake_avg, target_motor_torque);
  }
}