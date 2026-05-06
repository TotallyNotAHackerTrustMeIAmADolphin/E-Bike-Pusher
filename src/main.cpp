// main.cpp
#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include "dashboard.h"
#include "ODriveCAN.h"
#include "CadenceSensor.h"

ODriveCAN odrive(0);
CadenceSensor cadenceSensor;

#define EEPROM_SIZE 256
constexpr int EEPROM_ADDRESS = 0;

struct DeviceInfo {
  char macAddress[18];
  char deviceName[20];
  esp_ble_addr_type_t addressType;
  bool SCAN_FOR_DEVICE;
  int brakingThreshold;
  int brakingTimeout;
  char home_ssid[32];
  char home_pass[64];
};
DeviceInfo deviceInfo;

char log_buffer[1024] = "System Booting...\n";
void addLog(const char* msg) {
  Serial.println(msg);
  int msg_len = strlen(msg);
  int cur_len = strlen(log_buffer);
  
  if (cur_len + msg_len + 2 > sizeof(log_buffer)) {
    int shift = (cur_len + msg_len + 2) - sizeof(log_buffer);
    memmove(log_buffer, log_buffer + shift, cur_len - shift + 1);
    cur_len -= shift;
  }
  strcat(log_buffer, msg);
  strcat(log_buffer, "\n");
}

// --- HARDWARE PINS ---
#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_16
// FIX: Moved from 34 (Input Only) to 25 so the internal pull-up works!
#define inductiveProbe 25 

bool isBraking = false;
float brake_avg = 1.0; 
unsigned long brake_start_time = 0;

// --- OTA STATE & CALLBACKS ---
bool isUpdating = false;

void handleOTAStart() {
  isUpdating = true;
  odrive.setTorque(0.0); // Stop motor safely!
  Serial.println("\nOTA Update Started! Freezing all other tasks...");
}

void handleOTAEnd() {
  Serial.println("\nOTA Update Finished! Rebooting...");
}

void handleOTAError(ota_error_t error) {
  Serial.printf("\nOTA Error[%u]! Rebooting to recover...\n", error);
  ESP.restart(); // Force reboot to clear the frozen state!
}

const char *ap_ssid = "ESP-Cadence";
const char *ap_pass = "12345678";
bool isAPMode = false;
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
DNSServer dnsServer;
WebServer server(80);

void updateBrakeLogic() {
  int probe_state = (digitalRead(inductiveProbe) == HIGH) ? 1 : 0; 
  float alpha = 0.15; 
  brake_avg = (alpha * probe_state) + ((1.0 - alpha) * brake_avg);

  if (probe_state == 0) {
    if (brake_start_time == 0) brake_start_time = millis(); 
    if (millis() - brake_start_time > deviceInfo.brakingTimeout) {
      brake_avg = 0.0; 
      isBraking = true; 
    } else {
      isBraking = (brake_avg < 0.5); 
    }
  } else {
    brake_start_time = 0; 
    isBraking = (brake_avg < 0.5);
  }
}

// =======================================================
// WEB SERVER HANDLERS (Keep-Alive Enabled!)
// =======================================================
void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void handleData() {
  float power = abs((odrive.getCurrent() * 0.356) * (odrive.getVelocity() * 6.283185));
  char json[300];
  snprintf(json, sizeof(json), "{\"cadence\":%d, \"power\":%.1f, \"probe\":%.2f, \"isBraking\":%s, \"thresh\":%d, \"timeout\":%d, \"ssid\":\"%s\", \"psk\":\"%s\"}", 
           cadenceSensor.getCadence(), power, brake_avg, isBraking ? "true" : "false", deviceInfo.brakingThreshold, deviceInfo.brakingTimeout, deviceInfo.home_ssid, deviceInfo.home_pass);
  
  server.send(200, "application/json", json);
}

void handleLog() {
  server.send(200, "text/plain", log_buffer);
}

void handleSettings() {
  deviceInfo.brakingThreshold = server.arg("th").toInt();
  deviceInfo.brakingTimeout = server.arg("ti").toInt();
  EEPROM.put(EEPROM_ADDRESS, deviceInfo); EEPROM.commit();
  server.send(200, "text/plain", "OK");
  addLog("Brake settings updated.");
}

void handleWiFi() {
  strncpy(deviceInfo.home_ssid, server.arg("s").c_str(), 31);
  strncpy(deviceInfo.home_pass, server.arg("p").c_str(), 63);
  EEPROM.put(EEPROM_ADDRESS, deviceInfo); EEPROM.commit();
  server.send(200, "text/plain", "OK");
  delay(500); ESP.restart();
}

void handleScan() {
  deviceInfo.SCAN_FOR_DEVICE = true;
  EEPROM.put(EEPROM_ADDRESS, deviceInfo);
  EEPROM.commit();
  server.send(200, "text/plain", "Rebooting to scan...");
  delay(500);
  ESP.restart();
}

// =======================================================
// SETUP
// =======================================================

void setup() {
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRESS, deviceInfo);
  
  if(deviceInfo.brakingThreshold <= 0 || deviceInfo.brakingThreshold > 4096) {
      deviceInfo.brakingThreshold = 2048; deviceInfo.brakingTimeout = 2000;
      strncpy(deviceInfo.home_ssid, "wlesswg", 31);
      strncpy(deviceInfo.home_pass, "hba.1245", 63);
  }

  pinMode(inductiveProbe, INPUT_PULLUP);

  // --- Hybrid WiFi Setup ---
  addLog("Connecting to Home WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(deviceInfo.home_ssid, deviceInfo.home_pass);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 15) { delay(500); attempts++; }

  if (WiFi.status() == WL_CONNECTED) {
    // FORMAT AND PRINT THE IP ADDRESS!
    char msg[64];
    snprintf(msg, sizeof(msg), "Connected to Home! IP: %s", WiFi.localIP().toString().c_str());
    addLog(msg);
    isAPMode = false;
  } else {
    addLog("Home Network not found. Starting AP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(ap_ssid, ap_pass);
    dnsServer.start(DNS_PORT, "*", apIP);
    
    // FORMAT AND PRINT THE AP IP ADDRESS!
    char msg[64];
    snprintf(msg, sizeof(msg), "AP Started! IP: %s", WiFi.softAPIP().toString().c_str());
    addLog(msg);
    isAPMode = true;
  }
  
  // Attach the OTA safety callbacks!
  ArduinoOTA.setHostname("odrive-node");
  ArduinoOTA.onStart(handleOTAStart);
  ArduinoOTA.onEnd(handleOTAEnd);
  ArduinoOTA.onError(handleOTAError);
  ArduinoOTA.begin();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/log", HTTP_GET, handleLog);
  server.on("/api/settings", HTTP_POST, handleSettings);
  server.on("/api/wifi", HTTP_POST, handleWiFi);
  server.on("/api/scan", HTTP_POST, handleScan);
  server.begin();

  odrive.begin(CAN_TX_PIN, CAN_RX_PIN);
  odrive.setMode(1, 1); 
  delay(10);
  odrive.setState(8);

  cadenceSensor.begin(deviceInfo.SCAN_FOR_DEVICE, deviceInfo.macAddress, deviceInfo.addressType);
}

// =======================================================
// MAIN LOOP
// =======================================================
unsigned long last_cmd_time = 0;

void loop() {
  // If updating, ONLY handle OTA and skip absolutely everything else!
  if (isUpdating) {
    ArduinoOTA.handle();
    return; 
  }

  if (isAPMode) dnsServer.processNextRequest();
  server.handleClient();
  ArduinoOTA.handle();
  
  odrive.poll();
  cadenceSensor.loop();
  Flash this via USB. Open your Serial Moni
  updateBrakeLogic();

  if (cadenceSensor.foundNewDevice()) {
    strlcpy(deviceInfo.macAddress, cadenceSensor.getNewMac(), sizeof(deviceInfo.macAddress));
    strlcpy(deviceInfo.deviceName, cadenceSensor.getNewName(), sizeof(deviceInfo.deviceName));
    deviceInfo.addressType = cadenceSensor.getNewAddressType();
    
    // FIX: Tell the EEPROM to stop scanning on the next boot!
    deviceInfo.SCAN_FOR_DEVICE = false; 
    
    EEPROM.put(EEPROM_ADDRESS, deviceInfo);
    EEPROM.commit();
    cadenceSensor.clearNewDeviceFlag();
    addLog("Saved new BLE Sensor to EEPROM!");
  }

  if (millis() - last_cmd_time >= 20) {
    last_cmd_time = millis();
    float target_motor_torque = 0.0;

    if (isBraking) {
      target_motor_torque = 0.0; 
    } 
    else if (cadenceSensor.getCadence() > 0) {
      float CADENCE_TO_TORQUE_MULTIPLIER = 0.01; 
      float base_torque = cadenceSensor.getCadence() * CADENCE_TO_TORQUE_MULTIPLIER;
      target_motor_torque = base_torque * brake_avg;
    }

    odrive.setTorque(target_motor_torque);
    odrive.requestData(CMD_GET_ENCODER_ESTIMATES);
    odrive.requestData(CMD_GET_IQC);
  }
}