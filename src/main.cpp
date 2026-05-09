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

// --- DYNAMIC CONTROL STATE ---
float brake_avg = 1.0f;
float target_velocity = 0.0f;
int current_odrive_mode = 2; // 1 = Torque (Coast/Brake), 2 = Velocity (Push)

unsigned long last_cmd_time = 0;
unsigned long last_dashboard_time = 0;
bool isUpdating = false;

// --- DASHBOARD CALLBACKS ---
void triggerEEPROMSave()
{
  EEPROM.put(EEPROM_ADDRESS, deviceInfo);
  EEPROM.commit();
  addLog("Settings saved to EEPROM!");
}
void triggerOTA()
{
  deviceInfo.maintenanceMode = true;
  triggerEEPROMSave();
  addLog("Rebooting to OTA Mode...");
  delay(500);
  ESP.restart();
}
void triggerScan()
{
  deviceInfo.SCAN_FOR_DEVICE = true;
  triggerEEPROMSave();
  addLog("Rebooting to Scan...");
  delay(500);
  ESP.restart();
}
void triggerWiFiSave(String s, String p)
{
  strlcpy(deviceInfo.home_ssid, s.c_str(), sizeof(deviceInfo.home_ssid));
  strlcpy(deviceInfo.home_pass, p.c_str(), sizeof(deviceInfo.home_pass));
  triggerEEPROMSave();
  delay(500);
  ESP.restart();
}

// --- BRAKING FILTER ---
void updateBrakeLogic()
{
  int currentProbeAnalog = analogRead(inductiveProbe);
  float target_state = (currentProbeAnalog < 2000) ? -1.0f : 1.0f;

  float dt = 0.02f;
  float tau = deviceInfo.brakeTimeConstant;
  if (tau < 0.01f)
    tau = 0.01f;
  float alpha = dt / (tau + dt);

  brake_avg = (alpha * target_state) + ((1.0f - alpha) * brake_avg);
}

// --- OTA MODE ---
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

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRESS, deviceInfo);

  pinMode(pullupPowerPin, OUTPUT);
  digitalWrite(pullupPowerPin, HIGH);
  pinMode(inductiveProbe, INPUT);

  if (deviceInfo.torqueMultiplier < 0 || isnan(deviceInfo.torqueMultiplier) || deviceInfo.home_ssid[0] == 255)
  {
    deviceInfo.torqueMultiplier = 1.0; // Default Accel Rate
    deviceInfo.brakeTimeConstant = 1.0;
    strncpy(deviceInfo.home_ssid, "wlesswg", 31);
    strncpy(deviceInfo.home_pass, "hba.1245", 63);
    deviceInfo.maintenanceMode = false;
    EEPROM.put(EEPROM_ADDRESS, deviceInfo);
    EEPROM.commit();
  }

  if (deviceInfo.maintenanceMode)
    runMaintenanceMode();

  dash_begin();

  odrive.begin(CAN_TX_PIN, CAN_RX_PIN);
  delay(250);
  odrive.setMode(2, 1); // Boot into Velocity Mode
  delay(50);
  odrive.setVelocity(0.0);
  delay(10);
  odrive.setState(8);

  cadenceSensor.begin(deviceInfo.SCAN_FOR_DEVICE, deviceInfo.macAddress, deviceInfo.addressType);
}

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

  // --- 50Hz VELOCITY INTEGRATOR ---
  if (millis() - last_cmd_time >= 20)
  {
    last_cmd_time = millis();
    updateBrakeLogic();

    // Read the live physical speed of the wheel
    float actual_velocity = odrive.getVelocity();
    if (actual_velocity < 0.0f)
      actual_velocity = 0.0f;

    if (brake_avg < -0.5f)
    {
      // --- ZONE 1: ACTIVE BRAKING ---
      // Switch to Torque mode for proportional smooth braking
      if (current_odrive_mode != 1)
      {
        odrive.setMode(1, 1);
        current_odrive_mode = 1;
      }

      if (actual_velocity > 0.05f)
      {
        float brake_factor = (abs(brake_avg) - 0.5f) / 0.5f;
        odrive.setTorque(-15.0f * brake_factor); // Max regen limit: 15 Amps
      }
      else
      {
        odrive.setTorque(0.0f); // Stop braking at a standstill
      }

      // Keep the internal integrator synced so it doesn't jerk when you start pedaling again!
      target_velocity = actual_velocity;
    }
    else if (cadenceSensor.getCadence() == 0)
    {
      // --- ZONE 2: COASTING (No Pedaling) ---
      // Switch to Torque Mode to perfectly freewheel!
      if (current_odrive_mode != 1)
      {
        odrive.setMode(1, 1);
        current_odrive_mode = 1;
      }
      odrive.setTorque(0.0f);

      // Keep synced!
      target_velocity = actual_velocity;
    }
    else
    {
      // --- ZONE 3: VELOCITY SPEED-MATCHING (Pedaling) ---
      // Switch back to Velocity mode to push the bike!
      if (current_odrive_mode != 2)
      {
        odrive.setMode(2, 1);
        current_odrive_mode = 2;
      }

      float speed_change = 0.0f;
      float accel_rate = deviceInfo.torqueMultiplier; // Re-using this setting as Accel Rate!

      // Push Zone
      if (brake_avg > 0.1f)
      {
        speed_change = (brake_avg - 0.1f) * accel_rate * 0.02f; // dt = 0.02s
      }
      // Coast Zone (Trailer overrunning bike slightly, gently reduce speed)
      else if (brake_avg < -0.1f)
      {
        speed_change = (brake_avg + 0.1f) * accel_rate * 0.02f;
      }

      target_velocity += speed_change;

      // --- THE 28-INCH SPEED LIMIT (25 km/h) ---
      if (target_velocity < 0.0f)
        target_velocity = 0.0f;
      if (target_velocity > 3.2f)
        target_velocity = 3.2f;

      odrive.setVelocity(target_velocity);
    }

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

    if (deviceConnected)
    {
      char json[128];
      snprintf(json, sizeof(json), "{\"c\":%d,\"p\":%.1f,\"v\":%.1f,\"pr\":%.2f,\"tq\":%.2f,\"m\":%d}",
               cadenceSensor.getCadence(), mech_power, odrive.getVoltage(), brake_avg, target_velocity, current_odrive_mode);
      pTxCharacteristic->setValue((uint8_t *)json, strlen(json));
      pTxCharacteristic->notify();
    }
  }
}