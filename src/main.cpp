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
// BRAKING LOGIC (With Deadband and hardcoded threshold)
// =======================================================
void updateBrakeLogic()
{
  int currentProbeAnalog = analogRead(inductiveProbe);

  // HARDCODED THRESHOLD: 2000
  // -1.0 = compressed (braking), 1.0 = relaxed (pushing)
  float target_state = (currentProbeAnalog < 2000) ? -1.0f : 1.0f;

  float dt = 0.02f; // 50Hz
  float tau = deviceInfo.brakeTimeConstant;
  if (tau < 0.01f)
    tau = 0.01f;
  float alpha = dt / (tau + dt);

  brake_avg = (alpha * target_state) + ((1.0f - alpha) * brake_avg);
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

  pinMode(pullupPowerPin, OUTPUT);
  digitalWrite(pullupPowerPin, HIGH);
  pinMode(inductiveProbe, INPUT);

  // EEPROM Data Integrity Check (Updated for removed threshold)
  if (deviceInfo.torqueMultiplier < 0 || isnan(deviceInfo.torqueMultiplier) || deviceInfo.home_ssid[0] == 255)
  {
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
unsigned long last_cmd_time = 0;
unsigned long last_dashboard_time = 0;
bool isUpdating = false; // Required for the loop check

void loop()
{
  ArduinoOTA.handle();
  if (isUpdating)
  {
    delay(1);
    return;
  }

  odrive.poll();
  cadenceSensor.loop();
  dash_loop();

  if (cadenceSensor.foundNewDevice())
  {
    strlcpy(deviceInfo.macAddress, cadenceSensor.getNewMac(), sizeof(deviceInfo.macAddress));
    strlcpy(deviceInfo.deviceName, cadenceSensor.getNewName(), sizeof(deviceInfo.deviceName));
    deviceInfo.addressType = cadenceSensor.getNewAddressType();
    deviceInfo.SCAN_FOR_DEVICE = false;
    triggerEEPROMSave();
    cadenceSensor.clearNewDeviceFlag();
  }

  // --- MOTOR CONTROL (50Hz) ---
  if (millis() - last_cmd_time >= 20)
  {
    last_cmd_time = millis();
    updateBrakeLogic();

    target_motor_torque = 0.0f;

    // --- HYSTERESIS DEAD BAND LOGIC ---
    if (brake_avg > 0.1f)
    {
      // ZONE: PUSHING (0.1 to 1.0)
      if (cadenceSensor.getCadence() > 0)
      {
        float push_factor = (brake_avg - 0.1f) / 0.9f;
        target_motor_torque = deviceInfo.torqueMultiplier * push_factor;
      }
    }
    else if (brake_avg < -0.5f)
    {
      // ZONE: ACTIVE REGEN (-0.5 to -1.0)
      if (odrive.getVelocity() > 0.05f)
      {
        float brake_factor = (abs(brake_avg) - 0.5f) / 0.5f;
        target_motor_torque = (deviceInfo.torqueMultiplier * brake_factor) * -1.0f;
      }
    }

    odrive.setTorque(target_motor_torque);

    odrive.requestData(CMD_GET_ENCODER_ESTIMATES);
    odrive.requestData(CMD_GET_IQC);
    odrive.requestData(CMD_GET_VBUS_VOLTAGE);
  }

  // --- DASHBOARD TELEMETRY (2Hz) ---
  if (millis() - last_dashboard_time >= 500)
  {
    last_dashboard_time = millis();
    float current_amps = odrive.getCurrent();
    float velocity_revs = odrive.getVelocity();

    // Mechanical Power calculation
    float mech_power = abs((current_amps * 0.356) * (velocity_revs * 6.283185));

    // Send telemetry including the valuable Brake Factor (brake_avg)
    dash_sendTelemetry(cadenceSensor.getCadence(), mech_power, odrive.getVoltage(), current_amps, brake_avg, target_motor_torque);
  }
}