// CadenceSensor.cpp
#include "CadenceSensor.h"

// Standard BLE UUIDs for Cadence Sensors
static BLEUUID serviceUUID("00001816-0000-1000-8000-00805f9b34fb");
static BLEUUID notifyUUID("00002a5b-0000-1000-8000-00805f9b34fb");

// Global pointer to route BLE C-callbacks back into our C++ object
CadenceSensor* globalSensorInstance = nullptr;

// --- BLE Callbacks ---
void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
  if (globalSensorInstance) globalSensorInstance->_onNotify(data, length);
}
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) { if (globalSensorInstance) globalSensorInstance->_onConnect(); }
  void onDisconnect(BLEClient* pclient) { if (globalSensorInstance) globalSensorInstance->_onDisconnect(); }
};
class MyScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) { if (globalSensorInstance) globalSensorInstance->_onScanResult(dev); }
};

// --- Class Implementation ---
CadenceSensor::CadenceSensor() {
  globalSensorInstance = this;
  _connected = false;
  _newDeviceFound = false;
  _cadence = 0;
  _lastEventTime = 0;
  _prevCumulativeCrankRev = 0;
  _prevCrankTime = 0;
  _prevCrankStaleness = 0;
  _prevRPM = 0.0;
  _foundDevice = nullptr;
}

void CadenceSensor::begin(bool scanMode, const char* savedMac, esp_ble_addr_type_t savedType) {
  _scanMode = scanMode;
  _targetMac = String(savedMac);
  _targetType = savedType;

  BLEDevice::init("");
  
  if (_scanMode) {
    addLog("BLE Scanning started...");
    _scanner = BLEDevice::getScan();
    _scanner->setAdvertisedDeviceCallbacks(new MyScanCallback());
    _scanner->setActiveScan(true);
  }
}

void CadenceSensor::loop() {
  // 1. Connection Management (With a 5-second anti-spam delay!)
  static unsigned long last_connect_attempt = 0;

  if (!_connected && (millis() - last_connect_attempt > 5000)) {
    last_connect_attempt = millis();

    if (_scanMode) {
      addLog("Scanning for BLE sensor...");
      _scanner->start(5, false); // Scan for 5 seconds
      _scanner->stop();
      if (_foundDevice != nullptr) connectToServer();
    } else {
      if (_targetMac.length() == 17) {
        connectToServer();
      } else {
        addLog("No BLE MAC saved. Click Scan on Dashboard!");
      }
    }
  }

  // 2. Timeout Logic (Fixes the stuck cadence bug!)
  if (_cadence > 0 && (millis() - _lastEventTime > 2000)) {
    _cadence = 0;
    _prevRPM = 0.0;
  }
}

int CadenceSensor::getCadence() { return _cadence; }
bool CadenceSensor::isConnected() { return _connected; }
bool CadenceSensor::foundNewDevice() { return _newDeviceFound; }
const char* CadenceSensor::getNewMac() { return _newMac.c_str(); }
const char* CadenceSensor::getNewName() { return _newName.c_str(); }
esp_ble_addr_type_t CadenceSensor::getNewAddressType() { return _newType; }
void CadenceSensor::clearNewDeviceFlag() { _newDeviceFound = false; }

bool CadenceSensor::connectToServer() {
  if (_client == nullptr) {
    _client = BLEDevice::createClient();
    _client->setClientCallbacks(new MyClientCallback());
  }

  bool result = false;
  if (_scanMode && _foundDevice != nullptr) {
    addLog("Connecting to newly scanned sensor...");
    result = _client->connect(_foundDevice);
  } else if (!_scanMode && _targetMac.length() == 17) {
    addLog("Connecting to saved sensor...");
    result = _client->connect(BLEAddress(_targetMac.c_str()), _targetType);
  }

  if (result) {
    BLERemoteService* remoteService = _client->getService(serviceUUID);
    if (remoteService) {
      BLERemoteCharacteristic* sensorChar = remoteService->getCharacteristic(notifyUUID);
      if (sensorChar) sensorChar->registerForNotify(notifyCallback);
    }
  } else {
    addLog("BLE Connection failed. Retrying...");
  }
  return result;
}

void CadenceSensor::_onConnect() {
  addLog("BLE Sensor Connected!");
  _connected = true;
  if (_scanMode) {
    _scanMode = false;
    _newDeviceFound = true; 
    // Force safe string conversion so it doesn't vanish from memory
    _newMac = String(_foundDevice->getAddress().toString().c_str());
    _newName = String(_foundDevice->getName().c_str());
    _newType = (esp_ble_addr_type_t)_foundDevice->getAddressType();
  }
}

void CadenceSensor::_onDisconnect() {
  addLog("BLE Sensor Disconnected!");
  _connected = false;
}

void CadenceSensor::_onScanResult(BLEAdvertisedDevice dev) {
  if (dev.haveServiceUUID() && dev.isAdvertisingService(serviceUUID) && dev.getName().length() > 0) {
    if (_foundDevice == nullptr) _foundDevice = new BLEAdvertisedDevice(dev);
  }
}

void CadenceSensor::_onNotify(uint8_t* data, size_t length) {
  _lastEventTime = millis(); // Refresh the timeout watchdog

  bool hasWheel = (data[0] & 1);
  int cRevIdx = hasWheel ? 7 : 1, cTimeIdx = hasWheel ? 9 : 3;
  
  int cumCrankRev = (data[cRevIdx + 1] << 8) | data[cRevIdx];
  int lastCrankTime = (data[cTimeIdx + 1] << 8) | data[cTimeIdx];

  // FIX: 16-bit rollovers require adding exactly 65536
  int deltaRot = cumCrankRev - _prevCumulativeCrankRev; 
  if(deltaRot < 0) deltaRot += 65536;
  
  int timeDelta = lastCrankTime - _prevCrankTime;       
  if(timeDelta < 0) timeDelta += 65536;

  if (timeDelta > 0) {
    _prevCrankStaleness = 0;
    _prevRPM = ((double)deltaRot) / (((double)timeDelta) / 1024.0 / 60.0);
  } else if (_prevCrankStaleness < 2) {
    _prevCrankStaleness++;
  } else {
    _prevRPM = 0.0;
  }

  _prevCumulativeCrankRev = cumCrankRev;
  _prevCrankTime = lastCrankTime;
  _cadence = (_prevRPM > 160) ? 0 : (int)_prevRPM;
}