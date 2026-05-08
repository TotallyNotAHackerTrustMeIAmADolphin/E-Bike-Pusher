#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
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
  float torqueMultiplier;
  float brakeAlpha;
  char home_ssid[32];
  char home_pass[64];
  bool maintenanceMode; 
};
DeviceInfo deviceInfo;

// --- HARDWARE PINS ---
#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_16
#define inductiveProbe 34 
#define pullupPowerPin 33 

int currentProbeAnalog = 0; 
bool isBraking = false;
float brake_avg = 1.0; 
unsigned long brake_start_time = 0;

// --- BLE DASHBOARD SETUP ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_TX_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_RX_UUID        "828917c1-ea55-4d4a-a66e-fd202cea0645"
#define CHAR_LOG_UUID       "5111b1db-2917-48f8-b3d5-e51c8f35fc06" // NEW LOG CHANNEL

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pLogCharacteristic = NULL; // Pointer for sending logs
bool deviceConnected = false;

// --- LIVE LOGGING STREAM ---
// This function instantly pushes the text to the Serial Monitor AND your phone!
void addLog(const char* msg) {
  Serial.println(msg);
  if (deviceConnected && pLogCharacteristic != NULL) {
    pLogCharacteristic->setValue(msg);
    pLogCharacteristic->notify();
  }
}

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { 
      deviceConnected = true; 
      delay(500); // Give the phone a moment to subscribe before sending the welcome log!
      addLog("Phone Dashboard Connected!");
    }
    void onDisconnect(BLEServer* pServer) { 
        deviceConnected = false; 
        pServer->startAdvertising(); 
    }
};

class MyRxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue().c_str();
        if (rxValue.length() > 0) {
            Serial.print("Received Command: "); Serial.println(rxValue);
            
            if (rxValue == "OTA") {
                deviceInfo.maintenanceMode = true;
                EEPROM.put(EEPROM_ADDRESS, deviceInfo); EEPROM.commit();
                addLog("Rebooting to Maintenance Mode...");
                delay(500); ESP.restart();
            }
            else if (rxValue == "SCAN") {
                deviceInfo.SCAN_FOR_DEVICE = true;
                EEPROM.put(EEPROM_ADDRESS, deviceInfo); EEPROM.commit();
                addLog("Rebooting to Scan for Cadence Sensor...");
                delay(500); ESP.restart();
            }
            else if (rxValue.startsWith("SET:")) {
                int sep1 = rxValue.indexOf(':');
                int sep2 = rxValue.indexOf(':', sep1 + 1);
                String key = rxValue.substring(sep1 + 1, sep2);
                String val = rxValue.substring(sep2 + 1);
                
                if (key == "th") deviceInfo.brakingThreshold = val.toInt();
                if (key == "ti") deviceInfo.brakingTimeout = val.toInt();
                if (key == "tm") deviceInfo.torqueMultiplier = val.toFloat();
                if (key == "ba") deviceInfo.brakeAlpha = val.toFloat();
                
                EEPROM.put(EEPROM_ADDRESS, deviceInfo); EEPROM.commit();
                addLog("Tuning Parameters Saved!");
            }
        }
    }
};

void updateBrakeLogic() {
  currentProbeAnalog = analogRead(inductiveProbe);
  int probe_state = (currentProbeAnalog < deviceInfo.brakingThreshold) ? 0 : 1; 
  brake_avg = (deviceInfo.brakeAlpha * probe_state) + ((1.0 - deviceInfo.brakeAlpha) * brake_avg);

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
// MAINTENANCE MODE (WIFI + OTA ONLY!)
// =======================================================
void runMaintenanceMode() {
  Serial.println("MAINTENANCE MODE: Turning on WiFi for OTA Updates.");
  
  deviceInfo.maintenanceMode = false;
  EEPROM.put(EEPROM_ADDRESS, deviceInfo); EEPROM.commit();

  WiFi.mode(WIFI_STA);
  WiFi.begin(deviceInfo.home_ssid, deviceInfo.home_pass);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); attempts++; Serial.print("."); }

  DNSServer dnsServer;
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nConnected to Home WiFi! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nHome WiFi failed. Starting AP: ESP-Maintenance");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP-Maintenance", "12345678");
    dnsServer.start(53, "*", WiFi.softAPIP());
    Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
  }

  ArduinoOTA.setHostname("odrive-node");
  ArduinoOTA.begin();
  Serial.println("Ready for PlatformIO OTA Upload!");
  
  while (true) {
    ArduinoOTA.handle();
    dnsServer.processNextRequest();
    delay(2);
  }
}

// =======================================================
// SETUP (Normal Boot)
// =======================================================
void setup() {
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRESS, deviceInfo);
  
  if(deviceInfo.brakingThreshold <= 0 || deviceInfo.brakingThreshold > 4096 || isnan(deviceInfo.torqueMultiplier)) {
      deviceInfo.brakingThreshold = 2048; 
      deviceInfo.brakingTimeout = 2000;
      deviceInfo.torqueMultiplier = 0.01; 
      deviceInfo.brakeAlpha = 0.15;       
      strncpy(deviceInfo.home_ssid, "wlesswg", 31);
      strncpy(deviceInfo.home_pass, "hba.1245", 63);
      deviceInfo.maintenanceMode = false;
      EEPROM.put(EEPROM_ADDRESS, deviceInfo); EEPROM.commit();
  }

  if (deviceInfo.maintenanceMode) {
    runMaintenanceMode(); 
  }

  // --- NORMAL E-BIKE BOOT BELOW ---
  pinMode(pullupPowerPin, OUTPUT);
  digitalWrite(pullupPowerPin, HIGH); 
  pinMode(inductiveProbe, INPUT); 

  BLEDevice::init("E-Bike Pusher");
  BLEDevice::setMTU(512);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pTxCharacteristic = pService->createCharacteristic(CHAR_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHAR_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyRxCallbacks());

  // NEW: Setup the Live Log Characteristic
  pLogCharacteristic = pService->createCharacteristic(CHAR_LOG_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pLogCharacteristic->addDescriptor(new BLE2902());
  
  pService->start();
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();

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
unsigned long last_dashboard_time = 0;

void loop() {
  odrive.poll();
  cadenceSensor.loop();
  updateBrakeLogic();

  if (cadenceSensor.foundNewDevice()) {
    strlcpy(deviceInfo.macAddress, cadenceSensor.getNewMac(), sizeof(deviceInfo.macAddress));
    strlcpy(deviceInfo.deviceName, cadenceSensor.getNewName(), sizeof(deviceInfo.deviceName));
    deviceInfo.addressType = cadenceSensor.getNewAddressType();
    deviceInfo.SCAN_FOR_DEVICE = false; 
    EEPROM.put(EEPROM_ADDRESS, deviceInfo); EEPROM.commit();
    cadenceSensor.clearNewDeviceFlag();
    addLog("Cadence Sensor paired & saved!");
  }

  if (millis() - last_cmd_time >= 20) {
    last_cmd_time = millis();
    float target_motor_torque = 0.0;

    if (!isBraking && cadenceSensor.getCadence() > 0) {
      target_motor_torque = (cadenceSensor.getCadence() * deviceInfo.torqueMultiplier) * brake_avg;
    }

    odrive.setTorque(target_motor_torque);
    odrive.requestData(CMD_GET_ENCODER_ESTIMATES);
    odrive.requestData(CMD_GET_IQC);
  }

  // --- BLUETOOTH DASHBOARD NOTIFY (2Hz) ---
  if (deviceConnected && millis() - last_dashboard_time >= 500) {
    last_dashboard_time = millis();
    float power = abs((odrive.getCurrent() * 0.356) * (odrive.getVelocity() * 6.283185));
    
    char json[150];
    snprintf(json, sizeof(json), "{\"c\":%d,\"p\":%.1f,\"pr\":%d,\"b\":%d,\"th\":%d,\"ti\":%d,\"tm\":%.3f,\"ba\":%.2f}", 
             cadenceSensor.getCadence(), power, currentProbeAnalog, isBraking?1:0, 
             deviceInfo.brakingThreshold, deviceInfo.brakingTimeout, deviceInfo.torqueMultiplier, deviceInfo.brakeAlpha);
    
    pTxCharacteristic->setValue(json);
    pTxCharacteristic->notify();
  }
}