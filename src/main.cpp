#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include "ODriveCAN.h"
#include "CadenceSensor.h"
#include "BLEDashboard.h"

ODriveCAN odrive(0);
CadenceSensor cadenceSensor;
DeviceInfo deviceInfo;
Preferences preferences;
portMUX_TYPE deviceInfoMux = portMUX_INITIALIZER_UNLOCKED;

#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_16
#define inductiveProbe 34
#define pullupPowerPin 33

volatile float brake_avg = 1.0f;
volatile float dashboard_target_val = 0.0f;
volatile int current_odrive_mode = 2;
volatile bool isBraking = false;
volatile bool isUpdating = false;

unsigned long last_dashboard_time = 0;

void triggerEEPROMSave()
{
  preferences.begin("espcadence", false);
  preferences.putBytes("deviceInfo", &deviceInfo, sizeof(deviceInfo));
  preferences.end();
  addLog("Settings saved to Preferences!");
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

void updateBrakeLogic(float dt, float tau)
{
  // NOTE: The inductive sensor is a digital open-collector type that pulls to GND.
  // We use analogRead() because the voltage swing can be marginal/soft near the 
  // ESP32 logic thresholds. Analog reading allows for custom hysteresis.
  int currentProbeAnalog = analogRead(inductiveProbe);
  static float target_state = 1.0f;

  // Hysteresis: Prevents rapid toggling (chatter)
  if (currentProbeAnalog < 1800)
    target_state = -1.0f; // Compressed
  else if (currentProbeAnalog > 2200)
    target_state = 1.0f; // Extended

  // Sensor Safety: Dead-man range check
  if (currentProbeAnalog < 200 || currentProbeAnalog > 3900)
  {
    target_state = 0.0f; // Neutral (Safe)
    static unsigned long last_error_log = 0;
    if (millis() - last_error_log > 5000)
    {
      addLog("WARNING: Hitch Sensor Failure!");
      last_error_log = millis();
    }
  }

  if (tau < 0.01f)
    tau = 0.01f;
  float alpha = dt / (tau + dt);
  brake_avg = (alpha * target_state) + ((1.0f - alpha) * brake_avg);
}

void runMaintenanceMode()
{
  deviceInfo.maintenanceMode = false;
  preferences.begin("espcadence", false);
  preferences.putBytes("deviceInfo", &deviceInfo, sizeof(deviceInfo));
  preferences.end();
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

static void ensureTorqueControlMode()
{
  if (current_odrive_mode != 1)
  {
    odrive.setMode(1, 1);
    current_odrive_mode = 1;
  }
}

void vControlTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz

  // Register this task with the Watchdog
  esp_task_wdt_add(NULL);

  static float I_out = 0.0f;
  static float prev_error = 0.0f;
  static unsigned long last_revive_time = 0;

  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    esp_task_wdt_reset(); // Feed the Dog

    if (isUpdating)
      continue;

    const float dt = 0.020f; // Fixed 50Hz delta (guaranteed by vTaskDelayUntil)

    // Snapshot the BLE-tunable PID/filter gains together so a concurrent CFG:
    // write from the dashboard can't hand us a torn mix of old/new values.
    float kp, ki, kd, maxSpeed, tau;
    taskENTER_CRITICAL(&deviceInfoMux);
    kp = deviceInfo.vel_Kp;
    ki = deviceInfo.vel_Ki;
    kd = deviceInfo.vel_Kd;
    maxSpeed = deviceInfo.max_speed;
    tau = deviceInfo.brakeTimeConstant;
    taskEXIT_CRITICAL(&deviceInfoMux);

    odrive.poll();
    updateBrakeLogic(dt, tau);

    bool canFresh = odrive.isDataFresh();

    // ROBUST AUTO-REVIVE
    if (odrive.getState() != 8)
    {
      uint32_t err = odrive.getError();
      if (millis() - last_revive_time > 2000)
      {
        last_revive_time = millis();
        if (err == ODRV_ERROR_NONE || err == ODRV_ERROR_WATCHDOG_TIMER_EXPIRED)
        {
          odrive.clearErrors();
          vTaskDelay(pdMS_TO_TICKS(1));
          odrive.setState(8);
          addLog("ODrive Watchdog Recovered. Re-arming...");
        }
        else
        {
          char msg[64];
          snprintf(msg, sizeof(msg), "CRITICAL: ODrive Error 0x%04X!", err);
          addLog(msg);
        }
      }
    }

    float actual_velocity = odrive.getVelocity();
    if (actual_velocity < 0.0f)
      actual_velocity = 0.0f;

    if (!canFresh)
    {
      // Force Torque Control with 0A (same safe freewheel state as Zone 2) so a
      // stale Velocity Control command can't keep driving the motor.
      ensureTorqueControlMode();
      odrive.setTorque(0.0f);
      I_out = actual_velocity;
      prev_error = brake_avg;
      dashboard_target_val = 0.0f;
    }
    else if (brake_avg < -0.5f)
    {
      // --- ZONE 1: ACTIVE REGEN BRAKING ---
      isBraking = true;
      ensureTorqueControlMode();

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
      I_out = actual_velocity;
      prev_error = brake_avg;
    }
    else if (cadenceSensor.getCadence() == 0)
    {
      // --- ZONE 2: COASTING ---
      isBraking = false;
      ensureTorqueControlMode();
      odrive.setTorque(0.0f);
      I_out = actual_velocity;
      prev_error = brake_avg;
      dashboard_target_val = 0.0f;
    }
    else
    {
      // --- ZONE 3: PID VELOCITY PUSH ---
      isBraking = false;
      if (current_odrive_mode != 2)
      {
        odrive.setMode(2, 1);
        current_odrive_mode = 2;
        I_out = actual_velocity - (kp * brake_avg);
        prev_error = brake_avg;
      }

      float error = brake_avg;
      float P_out = kp * error;
      I_out += (ki * error) * dt;
      float derivative = (error - prev_error) / dt;
      float D_out = kd * derivative;
      prev_error = error;

      float target_velocity = I_out + P_out + D_out;

      if (target_velocity > maxSpeed)
      {
        target_velocity = maxSpeed;
        I_out = maxSpeed;
      }
      else if (target_velocity < 0.0f)
      {
        target_velocity = 0.0f;
        I_out = 0.0f;
      }

      odrive.setVelocity(target_velocity);
      dashboard_target_val = target_velocity;
    }

    odrive.requestData(CMD_GET_ENCODER_ESTIMATES);
    odrive.requestData(CMD_GET_IQC);
    odrive.requestData(CMD_GET_VBUS_VOLTAGE);
  }
}

void setup()
{
  Serial.begin(115200);

  // Initialize Preferences
  preferences.begin("espcadence", false);
  if (preferences.getBytesLength("deviceInfo") == sizeof(deviceInfo))
  {
    preferences.getBytes("deviceInfo", &deviceInfo, sizeof(deviceInfo));
  }
  else
  {
    deviceInfo.vel_Kp = 1.0;
    deviceInfo.vel_Ki = 2.0;
    deviceInfo.vel_Kd = 0.0;
    deviceInfo.max_speed = 10.0;
    deviceInfo.brakeTimeConstant = 1.0;
    strncpy(deviceInfo.home_ssid, "wlesswg", 31);
    strncpy(deviceInfo.home_pass, "hba.1245", 63);
    deviceInfo.maintenanceMode = false;
    deviceInfo.SCAN_FOR_DEVICE = false;
    deviceInfo.macAddress[0] = '\0';
    deviceInfo.deviceName[0] = '\0';
    deviceInfo.addressType = 0;
    preferences.putBytes("deviceInfo", &deviceInfo, sizeof(deviceInfo));
    Serial.println("Preferences Reset to defaults.");
  }
  preferences.end();

  pinMode(pullupPowerPin, OUTPUT);
  digitalWrite(pullupPowerPin, HIGH);
  pinMode(inductiveProbe, INPUT);

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

  ArduinoOTA.onStart([]() {
    isUpdating = true;
    odrive.setTorque(0.0f);
    odrive.setState(1); // Idle
  });
  ArduinoOTA.onEnd([]() {
    isUpdating = false;
  });

  // --- RTOS & Watchdog Initialization ---
  // Deliberately NOT calling esp_task_wdt_init() here: this build's sdkconfig
  // already brings up the shared Task Watchdog Timer at boot (5s, panic-enabled),
  // and CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y means the core-0 idle task is
  // already subscribed to it. Re-calling esp_task_wdt_init() with a shorter
  // timeout would tighten that SHARED timeout for every subscriber, not just
  // vControlTask, risking a spurious panic/reboot from an unrelated core-0 stall
  // (BLE stack work, flash writes, OTA polling) that used to have 5s of slack.
  // vControlTask still gets fast-panic protection by subscribing to the existing
  // watchdog via esp_task_wdt_add() below.
  BaseType_t controlTaskCreated = xTaskCreatePinnedToCore(vControlTask, "ControlTask", 4096, NULL, 3, NULL, 1);
  if (controlTaskCreated != pdPASS)
  {
    addLog("CRITICAL: Failed to create control task! Rebooting...");
    delay(100);
    ESP.restart();
  }
}

void loop()
{
  ArduinoOTA.handle();
  if (isUpdating)
  {
    delay(1);
    return;
  }

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

  if (millis() - last_dashboard_time >= 500)
  {
    last_dashboard_time = millis();
    float mech_power = abs((odrive.getCurrent() * 0.356) * (odrive.getVelocity() * 6.283185));
    dash_sendTelemetry(cadenceSensor.getCadence(), mech_power, odrive.getVoltage(), odrive.getCurrent(), (float)brake_avg, (float)dashboard_target_val, odrive.getVelocity(), (int)current_odrive_mode);
  }
}
