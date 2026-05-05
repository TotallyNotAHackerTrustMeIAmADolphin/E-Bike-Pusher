#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include "driver/twai.h"
#include "device.h"
#include "dashboard.h"

// --- EEPROM SETTINGS ---
#define EEPROM_SIZE 128
constexpr int EEPROM_ADDRESS = 0;

struct DeviceInfo {
  char macAddress[18];
  char deviceName[20];
  esp_ble_addr_type_t addressType;
  bool SCAN_FOR_DEVICE;
  int brakingThreshold;
  int brakingTimeout;
};
DeviceInfo deviceInfo;

// --- HARDWARE PINS ---
#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_16
#define inductiveProbe GPIO_NUM_34

// --- ODRIVE SETTINGS ---
#define ODRIVE_NODE_ID 0
#define CMD_SET_AXIS_STATE         0x07
#define CMD_SET_CONTROLLER_MODE    0x0B
#define CMD_SET_INPUT_VEL          0x0D
#define CMD_GET_ENCODER_ESTIMATES  0x09
#define CMD_GET_IQC                0x14

float odrv_vel = 0.0;
float odrv_current = 0.0;

// --- E-BIKE STATE ---
int cadence = 0;
bool isBraking = false;
int currentProbeAnalog = 0;

// --- BLE STATE ---
bool connected = false;
static int prevCumulativeCrankRev = 0;
static int prevCrankTime = 0;
static double prevRPM = 0;
static int prevCrankStaleness = 0;
static int stalenessLimit = 2;
BLEClient* client;
BLEScan* scanner;
BLERemoteCharacteristic* sensorCharacteristic;

// --- WIFI / HYBRID CONFIG ---
const char *home_ssid = "TellMyWifiLoveHer";
const char *home_pass = "stehtaufdemRouter.";

const char *ap_ssid = "ESP-Cadence";
const char *ap_pass = "12345678";

bool isAPMode = false;
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
DNSServer dnsServer;
WebServer server(80);


// =======================================================
// CAN & ODRIVE FUNCTIONS
// =======================================================
void twai_send(uint32_t id, uint8_t* data, uint8_t len) {
  twai_message_t msg = { .extd = 0, .rtr = 0, .data_length_code = len };
  msg.identifier = id;
  memcpy(msg.data, data, len);
  twai_transmit(&msg, 0);
}

void set_odrive_velocity(float velocity) {
  uint8_t data[8] = {0};
  memcpy(&data[0], &velocity, 4);
  twai_send((ODRIVE_NODE_ID << 5) | CMD_SET_INPUT_VEL, data, 8);
}

void request_can_data(uint8_t cmd_id) {
  twai_message_t msg = { .extd = 0, .rtr = 1, .data_length_code = 0 }; 
  msg.identifier = (ODRIVE_NODE_ID << 5) | cmd_id;
  twai_transmit(&msg, 0);
}

void poll_can_messages() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    if (msg.rtr) continue; 
    uint8_t cmd_id = msg.identifier & 0x1F;
    if (cmd_id == CMD_GET_ENCODER_ESTIMATES) memcpy(&odrv_vel, &msg.data[4], 4); 
    if (cmd_id == CMD_GET_IQC) memcpy(&odrv_current, &msg.data[4], 4); 
  }
}

// =======================================================
// BRAKING LOGIC
// =======================================================
bool detectBraking() {
  int timeoutTicks = deviceInfo.brakingTimeout / 20; 
  static uint32_t prevMillis = 0;
  static int16_t brakingCounter = 0;
  
  if (millis() - prevMillis > 10) {
    prevMillis = millis();
    currentProbeAnalog = analogRead(inductiveProbe);
    
    if (currentProbeAnalog < deviceInfo.brakingThreshold) {
      brakingCounter++;
      if (brakingCounter > timeoutTicks) { brakingCounter = timeoutTicks; isBraking = true; }
    } else {
      brakingCounter--;
      if (brakingCounter < -timeoutTicks) { brakingCounter = -timeoutTicks; isBraking = false; }
    }
  }
  return isBraking;
}

// =======================================================
// BLE LOGIC
// =======================================================
static void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
  bool hasWheel = (data[0] & 1);
  int crankRevIndex = hasWheel ? 7 : 1;
  int crankTimeIndex = hasWheel ? 9 : 3;

  int cumulativeCrankRev = (data[crankRevIndex + 1] << 8) | data[crankRevIndex];
  int lastCrankTime = (data[crankTimeIndex + 1] << 8) | data[crankTimeIndex];

  int deltaRotations = cumulativeCrankRev - prevCumulativeCrankRev;
  if (deltaRotations < 0) deltaRotations += 65535;

  int timeDelta = lastCrankTime - prevCrankTime;
  if (timeDelta < 0) timeDelta += 65535;

  if (timeDelta != 0) {
    prevCrankStaleness = 0;
    double timeMins = ((double)timeDelta) / 1024.0 / 60.0;
    prevRPM = ((double)deltaRotations) / timeMins;
  } else if (prevCrankStaleness < stalenessLimit) {
    prevCrankStaleness++;
  } else {
    prevRPM = 0.0;
  }

  prevCumulativeCrankRev = cumulativeCrankRev;
  prevCrankTime = lastCrankTime;
  cadence = (prevRPM > 160) ? 0 : (int)prevRPM;
}

class ClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("BLE Connected!");
    if (deviceInfo.SCAN_FOR_DEVICE) {
      deviceInfo.SCAN_FOR_DEVICE = false;
      EEPROM.put(EEPROM_ADDRESS, deviceInfo);
      EEPROM.commit();
    }
    connected = true;
  }
  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("BLE Disconnected!");
  }
};

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // FIX 1: Changed .size() to .length()
    if (advertisedDevice.getName().length() > 0) {
      BLEAdvertisedDevice* d = new BLEAdvertisedDevice(advertisedDevice);
      addDevice(d);
    }
  }
};

void connectToServer() {
  if(client == nullptr) {
    client = BLEDevice::createClient();
    client->setClientCallbacks(new ClientCallback());
  }

  bool result = false;
  if (deviceInfo.SCAN_FOR_DEVICE && selectDevice() != nullptr) {
    BLEAdvertisedDevice* dev = selectDevice();
    strncpy(deviceInfo.macAddress, dev->getAddress().toString().c_str(), 17);
    strncpy(deviceInfo.deviceName, dev->getName().c_str(), 19);
    
    // FIX 2: Cast the return type to esp_ble_addr_type_t
    deviceInfo.addressType = (esp_ble_addr_type_t)dev->getAddressType(); 
    
    result = client->connect(dev);
  } else if (!deviceInfo.SCAN_FOR_DEVICE && strlen(deviceInfo.macAddress) == 17) {
    BLEAddress bleAddress(deviceInfo.macAddress);
    result = client->connect(bleAddress, deviceInfo.addressType);
  }

  if (result) {
    BLERemoteService* remoteService = client->getService(serviceUUID);
    if (remoteService) {
      sensorCharacteristic = remoteService->getCharacteristic(notifyUUID);
      if (sensorCharacteristic) sensorCharacteristic->registerForNotify(notifyCallback);
    }
  }
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRESS, deviceInfo);
  
  if(deviceInfo.brakingThreshold <= 0 || deviceInfo.brakingThreshold > 4096) {
      deviceInfo.brakingThreshold = 2048;
      deviceInfo.brakingTimeout = 2000;
  }

  pinMode(inductiveProbe, INPUT_PULLUP);

  // --- HYBRID WIFI SETUP ---
  Serial.println("\nAttempting to connect to Home WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(home_ssid, home_pass);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Home Network!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    isAPMode = false;
  } else {
    Serial.println("\nHome network not found. Starting Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(ap_ssid, ap_pass);
    dnsServer.start(DNS_PORT, "*", apIP);
    Serial.print("AP Started! Connect to ESP-Cadence. IP: ");
    Serial.println(WiFi.softAPIP());
    isAPMode = true;
  }
  
  ArduinoOTA.setHostname("odrive-node");
  ArduinoOTA.begin();

  // --- Web Server Setup ---
  server.on("/", HTTP_GET,[]() { server.send(200, "text/html", index_html); });
  
  server.on("/api/data", HTTP_GET,[]() {
    float power = abs((odrv_current * 0.356) * (odrv_vel * 6.283185));
    char json[200];
    snprintf(json, sizeof(json), "{\"cadence\":%d, \"power\":%.1f, \"probe\":%d, \"isBraking\":%s, \"thresh\":%d, \"timeout\":%d}", 
             cadence, power, currentProbeAnalog, isBraking ? "true" : "false", deviceInfo.brakingThreshold, deviceInfo.brakingTimeout);
    server.send(200, "application/json", json);
  });

  server.on("/api/settings", HTTP_POST,[]() {
    deviceInfo.brakingThreshold = server.arg("th").toInt();
    deviceInfo.brakingTimeout = server.arg("ti").toInt();
    EEPROM.put(EEPROM_ADDRESS, deviceInfo);
    EEPROM.commit();
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/scan", HTTP_POST,[]() {
    deviceInfo.SCAN_FOR_DEVICE = true;
    EEPROM.put(EEPROM_ADDRESS, deviceInfo);
    EEPROM.commit();
    server.send(200, "text/plain", "Rebooting...");
    delay(500);
    ESP.restart();
  });

  server.begin();

  // --- CAN & ODrive Setup ---
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS(); 
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  uint32_t state = 8; 
  uint8_t mode[8] = {2, 0, 0, 0, 1, 0, 0, 0}; 
  twai_send((ODRIVE_NODE_ID << 5) | CMD_SET_CONTROLLER_MODE, mode, 8);
  twai_send((ODRIVE_NODE_ID << 5) | CMD_SET_AXIS_STATE, (uint8_t*)&state, 4);

  // --- BLE Setup ---
  BLEDevice::init("");
  if(deviceInfo.SCAN_FOR_DEVICE) {
    scanner = BLEDevice::getScan();
    scanner->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
    scanner->setActiveScan(true);
  }
}

// =======================================================
// MAIN LOOP
// =======================================================
unsigned long last_cmd_time = 0;

void loop() {
  if (isAPMode) dnsServer.processNextRequest();
  
  server.handleClient();
  ArduinoOTA.handle();
  poll_can_messages();
  
  detectBraking();

  // --- BLE Connection Handler ---
  if (deviceInfo.SCAN_FOR_DEVICE && !connected) {
    set_odrive_velocity(0.0); 
    scanner->start(11, false);
    BLEDevice::getScan()->stop();
    if (selectDevice() != nullptr) connectToServer();
  } 
  else if (!deviceInfo.SCAN_FOR_DEVICE && !connected) {
    connectToServer();
  }

  // --- MOTOR CONTROL (50Hz) ---
  if (millis() - last_cmd_time >= 20) {
    last_cmd_time = millis();
    
    float target_motor_velocity = 0.0;

    if (isBraking) {
      target_motor_velocity = 0.0; 
    } 
    else if (cadence > 0) {
      float CADENCE_MULTIPLIER = 0.05; 
      target_motor_velocity = cadence * CADENCE_MULTIPLIER;
    }

    set_odrive_velocity(target_motor_velocity);

    request_can_data(CMD_GET_ENCODER_ESTIMATES);
    request_can_data(CMD_GET_IQC);
  }
}