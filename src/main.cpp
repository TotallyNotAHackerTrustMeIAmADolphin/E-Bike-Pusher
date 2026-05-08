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

// --- HARDWARE PINS ---
#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_16
#define inductiveProbe 34 
#define pullupPowerPin 33 

int currentProbeAnalog = 0; 
bool isBraking = false;
float brake_avg = 1.0; 
unsigned long brake_start_time = 0;

// =======================================================
// CALLBACKS FROM BLE DASHBOARD
// =======================================================
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

// =======================================================
// BRAKING LOGIC
// =======================================================
void updateBrakeLogic() {
  currentProbeAnalog = analogRead(inductiveProbe);
  int probe_state = (currentProbeAnalog < deviceInfo.brakingThreshold) ? 0 : 1; 
  brake_avg = (deviceInfo.brakeAlpha * probe_state) + ((1.0 - deviceInfo.brakeAlpha) * brake_avg);

  if (probe_state == 0) {
    if (brake_start_time == 0) brake_start_time = millis(); 
    if (millis() - brake_start_time > deviceInfo.brakingTimeout) {
      brake_avg = 0.0; isBraking = true; 
    } else {
      isBraking = (brake_avg < 0.5); 
    }
  } else {
    brake_start_time = 0; isBraking = (brake_avg < 0.5);
  }
}

// =======================================================
// OTA MAINTENANCE MODE
// =======================================================
void runMaintenanceMode() {
  Serial.println("MAINTENANCE MODE: Turning on WiFi for OTA.");
  deviceInfo.maintenanceMode = false; triggerEEPROMSave();

  WiFi.mode(WIFI_STA);
  WiFi.begin(deviceInfo.home_ssid, deviceInfo.home_pass);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); attempts++; }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_AP); WiFi.softAP("ESP-Maintenance", "12345678");
  }

  ArduinoOTA.setHostname("odrive-node");
  ArduinoOTA.begin();
  
  while (true) { ArduinoOTA.handle(); delay(2); }
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRESS, deviceInfo);
  
  if(deviceInfo.brakingThreshold <= 0 || deviceInfo.brakingThreshold > 4096 || isnan(deviceInfo.torqueMultiplier)) {
      deviceInfo.brakingThreshold = 2048; 
      deviceInfo.brakingTimeout = 2000;
      deviceInfo.torqueMultiplier = 5.0; 
      deviceInfo.brakeAlpha = 0.15;       
      strncpy(deviceInfo.home_ssid, "wlesswg", 31);
      strncpy(deviceInfo.home_pass, "hba.1245", 63);
      deviceInfo.maintenanceMode = false;
      EEPROM.put(EEPROM_ADDRESS, deviceInfo); EEPROM.commit();
  }

  if (deviceInfo.maintenanceMode) runMaintenanceMode(); 

  pinMode(pullupPowerPin, OUTPUT); digitalWrite(pullupPowerPin, HIGH); 
  pinMode(inductiveProbe, INPUT); 

  dash_begin();
  
  // --- ODRIVE BOOT SEQUENCE ---
  odrive.begin(CAN_TX_PIN, CAN_RX_PIN);
  delay(250); // CRITICAL: Give the physical CAN transceiver time to boot!
  
  odrive.setTorque(0.0);   // Force 0 torque
  odrive.setVelocity(0.0); // Force 0 velocity just in case
  delay(10);
  
  odrive.setMode(1, 1);    // 1 = Torque Control, 1 = Passthrough
  delay(50);               // Let the ODrive process the mode change
  
  odrive.setState(8);      // NOW it is safe to enter Closed Loop!

  cadenceSensor.begin(deviceInfo.SCAN_FOR_DEVICE, deviceInfo.macAddress, deviceInfo.addressType);
}

// =======================================================
// MAIN LOOP
// =======================================================
unsigned long last_cmd_time = 0;
unsigned long last_dashboard_time = 0;

void loop() {
  odrive.poll();
  cadenceSensor.loop();
  dash_loop();
  updateBrakeLogic();

  if (cadenceSensor.foundNewDevice()) {
    strlcpy(deviceInfo.macAddress, cadenceSensor.getNewMac(), sizeof(deviceInfo.macAddress));
    strlcpy(deviceInfo.deviceName, cadenceSensor.getNewName(), sizeof(deviceInfo.deviceName));
    deviceInfo.addressType = cadenceSensor.getNewAddressType();
    deviceInfo.SCAN_FOR_DEVICE = false; 
    triggerEEPROMSave();
    cadenceSensor.clearNewDeviceFlag();
    addLog("Cadence Sensor paired & saved!");
  }

  // --- MOTOR CONTROL (50Hz) ---
  if (millis() - last_cmd_time >= 20) {
    last_cmd_time = millis();
    
    // NOTE: Because we enabled the ODrive Watchdog, this 50Hz CAN message 
    // keeps the motor alive! If ESP32 crashes, it will stop automatically.
    float target_motor_torque = 0.0;

    if (!isBraking && cadenceSensor.getCadence() > 0) {
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
    
    dash_sendTelemetry(cadenceSensor.getCadence(), mech_power, odrive.getVoltage(), odrive.getCurrent(), brake_avg, isBraking);
  }
}