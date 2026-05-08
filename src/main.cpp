#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include "ODriveCAN.h"
#include "CadenceSensor.h"
#include "BLEDashboard.h"

// --- HARDWARE OBJECTS ---
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

// --- GLOBAL STATE ---
bool isBraking = false;
float brake_avg = 1.0f; // Varies from -1.0 (Braking) to 1.0 (Pulling)
unsigned long last_cmd_time = 0;
unsigned long last_dashboard_time = 0;
bool isUpdating = false;
float target_motor_torque = 0.0f;

// =======================================================
// CALLBACKS FROM BLE DASHBOARD
// =======================================================
void triggerEEPROMSave()
{
  EEPROM.put(EEPROM_ADDRESS, deviceInfo);
  EEPROM.commit();
  addLog("Settings saved to EEPROM!");
}
void triggerOTA()
{
  deviceInfo.maintenanceMode = true;
  EEPROM.put(EEPROM_ADDRESS, deviceInfo);
  EEPROM.commit();
  addLog("Rebooting to OTA Mode...");
  delay(500);
  ESP.restart();
}
void triggerScan()
{
  deviceInfo.SCAN_FOR_DEVICE = true;
  EEPROM.put(EEPROM_ADDRESS, deviceInfo);
  EEPROM.commit();
  addLog("Rebooting to Scan...");
  delay(500);
  ESP.restart();
}
void triggerWiFiSave(String s, String p)
{
  strlcpy(deviceInfo.home_ssid, s.c_str(), sizeof(deviceInfo.home_ssid));
  strlcpy(deviceInfo.home_pass, p.c_str(), sizeof(deviceInfo.home_pass));
  EEPROM.put(EEPROM_ADDRESS, deviceInfo);
  EEPROM.commit();
  addLog("WiFi Saved! Rebooting...");
  delay(500);
  ESP.restart();
}

// =======================================================
// BRAKING LOGIC (Low Pass Filter P-Controller)
// =======================================================
void updateBrakeLogic()
{
  int currentProbeAnalog = analogRead(inductiveProbe);

  // -1.0 = compressed (braking), 1.0 = relaxed (pushing)
  float target_state = (currentProbeAnalog < deviceInfo.brakingThreshold) ? -1.0f : 1.0f;

  // Calculate Alpha based on the Time Constant (tau) from the Dashboard
  float dt = 0.02f; // We run this at 50Hz
  float tau = deviceInfo.brakeTimeConstant;
  if (tau < 0.01f)
    tau = 0.01f; // Safety clamp
  float alpha = dt / (tau + dt);

  // Apply Exponential Moving Average
  brake_avg = (alpha * target_state) + ((1.0f - alpha) * brake_avg);

  // If average is negative, the "Timeout" has naturally passed and we are braking
  isBraking = (brake_avg < 0.0f);
}

// =======================================================
// OTA MAINTENANCE MODE
// =======================================================
void runMaintenanceMode()
{
  deviceInfo.maintenanceMode = false;
  EEPROM.put(EEPROM_ADDRESS, deviceInfo);
  EEPROM.commit();

  WiFi.mode(WIFI_STA);
  WiFi.begin(deviceInfo.home_ssid, deviceInfo.home_pass);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 15)
  {
    delay(500);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    char m[64];
    snprintf(m, sizeof(m), "OTA Ready! IP: %s", WiFi.localIP().toString().c_str());
    addLog(m);
  }
  else
  {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP-Maintenance", "12345678");
  }

  ArduinoOTA.setHostname("odrive-node");
  ArduinoOTA.begin();
  while (true)
  {
    ArduinoOTA.handle();
    delay(2);
  }
}

// =======================================================
// SETUP
// =======================================================
void setup()
{
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRESS, deviceInfo);

  // Setup hardware bus first
  pinMode(pullupPowerPin, OUTPUT);
  digitalWrite(pullupPowerPin, HIGH);
  pinMode(inductiveProbe, INPUT);

  // EEPROM Data Integrity Check
  if (deviceInfo.brakingThreshold <= 0 || deviceInfo.brakingThreshold > 4096 || isnan(deviceInfo.torqueMultiplier))
  {
    deviceInfo.brakingThreshold = 2048;
    deviceInfo.torqueMultiplier = 1.0;  // 1 Amp default
    deviceInfo.brakeTimeConstant = 0.5; // 0.5s default smoothing
    strncpy(deviceInfo.home_ssid, "wlesswg", 31);
    strncpy(deviceInfo.home_pass, "hba.1245", 63);
    deviceInfo.maintenanceMode = false;
    EEPROM.put(EEPROM_ADDRESS, deviceInfo);
    EEPROM.commit();
    Serial.println("EEPROM Reset to defaults.");
  }

  if (deviceInfo.maintenanceMode)
    runMaintenanceMode();

  dash_begin();

  odrive.begin(CAN_TX_PIN, CAN_RX_PIN);
  delay(250);
  odrive.setMode(1, 1);
  delay(50);
  odrive.setTorque(0.0);
  delay(10);
  odrive.setState(8);

  cadenceSensor.begin(deviceInfo.SCAN_FOR_DEVICE, deviceInfo.macAddress, deviceInfo.addressType);
  addLog("System Online.");
}

// =======================================================
// MAIN LOOP
// =======================================================
void loop()
{
  // Always handle OTA requests
  ArduinoOTA.handle();
  if (isUpdating)
  {
    delay(1);
    return;
  }

  // Background Tasks
  odrive.poll();
  cadenceSensor.loop();
  dash_loop();

  // Handle New Cadence Sensor pairing
  if (cadenceSensor.foundNewDevice())
  {
    strlcpy(deviceInfo.macAddress, cadenceSensor.getNewMac(), sizeof(deviceInfo.macAddress));
    strlcpy(deviceInfo.deviceName, cadenceSensor.getNewName(), sizeof(deviceInfo.deviceName));
    deviceInfo.addressType = cadenceSensor.getNewAddressType();
    deviceInfo.SCAN_FOR_DEVICE = false;
    triggerEEPROMSave();
    cadenceSensor.clearNewDeviceFlag();
  }

  // --- 50Hz CONTROL LOOP ---
  if (millis() - last_cmd_time >= 20)
  {
    last_cmd_time = millis();
    updateBrakeLogic();

    target_motor_torque = 0.0f;

    if (!isBraking)
    {
      // POSITIVE REGION: Pushing the bike. Only if pedaling!
      if (cadenceSensor.getCadence() > 0)
      {
        target_motor_torque = deviceInfo.torqueMultiplier * brake_avg;
      }
    }
    else
    {
      // NEGATIVE REGION: Active braking.
      // Safety: Only regen if the wheel is actually spinning forward!
      if (odrive.getVelocity() > 0.05f)
      {
        target_motor_torque = deviceInfo.torqueMultiplier * brake_avg;
      }
    }

    odrive.setTorque(target_motor_torque);

    // Telemetry Requests
    odrive.requestData(CMD_GET_ENCODER_ESTIMATES);
    odrive.requestData(CMD_GET_IQC);
    odrive.requestData(CMD_GET_VBUS_VOLTAGE);
  }

  // --- 2Hz DASHBOARD UPDATE ---
  if (millis() - last_dashboard_time >= 500)
  {
    last_dashboard_time = millis();
    float mech_power = abs((odrive.getCurrent() * 0.356) * (odrive.getVelocity() * 6.283185));
    dash_sendTelemetry(cadenceSensor.getCadence(), mech_power, odrive.getVoltage(), odrive.getCurrent(), brake_avg, target_motor_torque);
  }
}