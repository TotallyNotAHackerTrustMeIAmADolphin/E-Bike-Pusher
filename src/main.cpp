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
float dashboard_target_val = 0.0f;
int current_odrive_mode = 2;
bool isBraking = false;
bool isUpdating = false;

unsigned long last_cmd_time = 0;
unsigned long last_dashboard_time = 0;

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
  addLog("WiFi Saved! Rebooting...");
  delay(500);
  ESP.restart();
}

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

  // Reset check for new PI variables
  if (isnan(deviceInfo.vel_Kp) || isnan(deviceInfo.vel_Ki) || deviceInfo.home_ssid[0] == 255)
  {
    deviceInfo.vel_Kp = 1.0;     // Instant punch
    deviceInfo.vel_Ki = 2.0;     // Accel rate over time
    deviceInfo.max_speed = 15.0; // Increased for geared hubs
    deviceInfo.brakeTimeConstant = 1.0;
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
  odrive.setMode(2, 1);
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

  // --- 50Hz PI VELOCITY CONTROLLER ---
  if (millis() - last_cmd_time >= 20)
  {
    last_cmd_time = millis();
    updateBrakeLogic();

    float actual_velocity = odrive.getVelocity();
    if (actual_velocity < 0.0f)
      actual_velocity = 0.0f;

    // I_out tracks the built-up speed memory
    static float I_out = 0.0f;

    if (brake_avg < -0.5f)
    {
      // --- ZONE 1: ACTIVE BRAKING ---
      isBraking = true;
      if (current_odrive_mode != 1)
      {
        odrive.setMode(1, 1);
        current_odrive_mode = 1;
      }

      if (actual_velocity > 0.05f)
      {
        float brake_factor = (abs(brake_avg) - 0.5f) / 0.5f;
        float regen_torque = -15.0f * brake_factor;
        odrive.setTorque(regen_torque);
        dashboard_target_val = regen_torque;
      }
      else
      {
        odrive.setTorque(0.0f);
        dashboard_target_val = 0.0f;
      }

      // Crucial: Anchor the integrator so we resume pedaling cleanly!
      I_out = actual_velocity;
    }
    else if (cadenceSensor.getCadence() == 0 || brake_avg <= 0.1f)
    {
      // --- ZONE 2: COASTING & DEADBAND ---
      isBraking = false;
      if (current_odrive_mode != 1)
      {
        odrive.setMode(1, 1);
        current_odrive_mode = 1;
      }

      odrive.setTorque(0.0f);
      I_out = actual_velocity; // Anchor the integrator
      dashboard_target_val = 0.0f;
    }
    else
    {
      // --- ZONE 3: VELOCITY PI PUSH ---
      isBraking = false;
      if (current_odrive_mode != 2)
      {
        odrive.setMode(2, 1);
        current_odrive_mode = 2;
      }

      // Map 0.1->1.0 to an error value of 0.0->1.0
      float error = (brake_avg - 0.1f) / 0.9f;

      // P-Term: Instant punch response!
      float P_out = deviceInfo.vel_Kp * error;

      // I-Term: Slow build-up acceleration!
      I_out += (deviceInfo.vel_Ki * error) * 0.02f; // dt = 0.02s

      float target_velocity = I_out + P_out;

      // Speed Limit clamp!
      if (target_velocity > deviceInfo.max_speed)
      {
        target_velocity = deviceInfo.max_speed;
        I_out = deviceInfo.max_speed; // Anti-windup
      }

      odrive.setVelocity(target_velocity);
      dashboard_target_val = target_velocity;
    }

    odrive.requestData(CMD_GET_ENCODER_ESTIMATES);
    odrive.requestData(CMD_GET_IQC);
    odrive.requestData(CMD_GET_VBUS_VOLTAGE);
  }

  // --- 2Hz DASHBOARD UPDATE ---
  if (millis() - last_dashboard_time >= 500)
  {
    last_dashboard_time = millis();
    float mech_power = abs((odrive.getCurrent() * 0.356) * (odrive.getVelocity() * 6.283185));

    // Sends: Cadence, Power, Vbus, Amps, Brake Filter, Target (Torque or Vel), Actual Vel, Mode
    dash_sendTelemetry(cadenceSensor.getCadence(), mech_power, odrive.getVoltage(), odrive.getCurrent(), brake_avg, dashboard_target_val, odrive.getVelocity(), current_odrive_mode);
  }
}