#include "CadenceSensor.h"

static NimBLEUUID serviceUUID("00001816-0000-1000-8000-00805f9b34fb");
static NimBLEUUID notifyUUID("00002a5b-0000-1000-8000-00805f9b34fb");

CadenceSensor *globalSensorInstance = nullptr;

void notifyCallback(NimBLERemoteCharacteristic *pChar, uint8_t *data, size_t length, bool isNotify)
{
  if (globalSensorInstance)
    globalSensorInstance->_onNotify(data, length);
}

class MyClientCallback : public NimBLEClientCallbacks
{
  void onConnect(NimBLEClient *pclient)
  {
    if (globalSensorInstance)
      globalSensorInstance->_onConnect();
  }
  void onDisconnect(NimBLEClient *pclient)
  {
    if (globalSensorInstance)
      globalSensorInstance->_onDisconnect();
  }
};

class MyScanCallback : public NimBLEAdvertisedDeviceCallbacks
{
  void onResult(NimBLEAdvertisedDevice *dev)
  {
    if (globalSensorInstance)
      globalSensorInstance->_onScanResult(dev);
  }
};

void scanEndedCB(NimBLEScanResults results)
{
  if (globalSensorInstance)
    globalSensorInstance->_onScanComplete();
}

CadenceSensor::CadenceSensor()
{
  globalSensorInstance = this;
  _connected = false;
  _newDeviceFound = false;
  _isScanning = false;
  _scanFinished = false;
  _isConnecting = false;
  _cadence = 0;
  _lastEventTime = 0;
  _prevCumulativeCrankRev = 0;
  _prevCrankTime = 0;
  _prevCrankStaleness = 0;
  _prevRPM = 0.0;
  _foundDevice = nullptr;
}

void CadenceSensor::begin(bool scanMode, const char *savedMac, uint8_t savedType)
{
  _scanMode = scanMode;
  _targetMac = String(savedMac);
  _targetType = savedType;

  _scanner = NimBLEDevice::getScan();
  _scanner->setAdvertisedDeviceCallbacks(new MyScanCallback());
  _scanner->setActiveScan(true);

  // THE FIX: Relax the scanner duty cycle!
  // 100ms interval, 40ms window = 40% duty cycle.
  // Leaves 60% of the radio free for the Dashboard!
  _scanner->setInterval(100);
  _scanner->setWindow(40);
}

void CadenceSensor::_onScanComplete()
{
  _isScanning = false;
  _scanFinished = true;
}

void CadenceSensor::loop()
{
  static unsigned long last_connect_attempt = 0;

  // 1. Process Finished Scans (Main Thread)
  if (_scanFinished)
  {
    _scanFinished = false;
    if (_foundDevice != nullptr)
    {
      _isConnecting = true;
      connectToServer();
      _isConnecting = false;
    }
    else
    {
      if (!_scanMode)
        addLog("Sensor asleep. Spin crank to wake it up!");
    }
    last_connect_attempt = millis();
  }

  // 2. Start new scans if needed
  if (!_connected && !_isScanning && !_isConnecting && (millis() - last_connect_attempt > 4000))
  {
    last_connect_attempt = millis();
    _isScanning = true;

    // THE FIX: Explicitly flush the old results to prevent internal memory leaks!
    _scanner->clearResults();

    if (_foundDevice != nullptr)
    {
      delete _foundDevice;
      _foundDevice = nullptr;
    }

    if (_scanMode)
      addLog("Scanning for ANY Cadence Sensor...");
    else
      addLog("Searching for saved Cadence Sensor...");

    _scanner->start(2, scanEndedCB, false);
  }

  // 3. Cadence Timeout Logic
  if (_cadence > 0 && (millis() - _lastEventTime > 2000))
  {
    _cadence = 0;
    _prevRPM = 0.0;
  }
}

bool CadenceSensor::connectToServer()
{
  if (_client == nullptr)
  {
    _client = NimBLEDevice::createClient();
    _client->setClientCallbacks(new MyClientCallback());
  }

  bool result = false;
  if (_scanMode && _foundDevice != nullptr)
  {
    addLog("Device Found! Connecting...");
    result = _client->connect(_foundDevice);
  }
  else if (!_scanMode && _targetMac.length() == 17)
  {
    addLog("Connecting to saved sensor...");
    NimBLEAddress savedAddr(_targetMac.c_str(), _targetType);
    result = _client->connect(savedAddr);
  }

  if (result)
  {
    NimBLERemoteService *remoteService = _client->getService(serviceUUID);
    if (remoteService)
    {
      NimBLERemoteCharacteristic *sensorChar = remoteService->getCharacteristic(notifyUUID);
      if (sensorChar)
      {
        if (sensorChar->canNotify())
        {
          sensorChar->subscribe(true, notifyCallback);
        }
      }
    }
  }
  else
  {
    addLog("BLE Connection failed. Retrying...");
  }
  return result;
}

void CadenceSensor::_onScanResult(NimBLEAdvertisedDevice *dev)
{
  if (dev->haveServiceUUID() && dev->isAdvertisingService(serviceUUID) && dev->getName().length() > 0)
  {
    if (_scanMode)
    {
      if (_foundDevice == nullptr)
        _foundDevice = new NimBLEAdvertisedDevice(*dev);
    }
    else
    {
      if (String(dev->getAddress().toString().c_str()).equalsIgnoreCase(_targetMac))
      {
        if (_foundDevice == nullptr)
          _foundDevice = new NimBLEAdvertisedDevice(*dev);
      }
    }
  }
}

void CadenceSensor::_onConnect()
{
  addLog("Cadence Sensor Connected!");
  _connected = true;
  if (_scanMode)
  {
    _scanMode = false;
    _newDeviceFound = true;
    _newMac = String(_foundDevice->getAddress().toString().c_str());
    _newName = String(_foundDevice->getName().c_str());
    _newType = _foundDevice->getAddress().getType();
  }
}

void CadenceSensor::_onDisconnect()
{
  addLog("Cadence Sensor Disconnected!");
  _connected = false;
}

void CadenceSensor::_onNotify(uint8_t *data, size_t length)
{
  _lastEventTime = millis();
  bool hasWheel = (data[0] & 1);
  int cRevIdx = hasWheel ? 7 : 1, cTimeIdx = hasWheel ? 9 : 3;
  int cumCrankRev = (data[cRevIdx + 1] << 8) | data[cRevIdx];
  int lastCrankTime = (data[cTimeIdx + 1] << 8) | data[cTimeIdx];

  int deltaRot = cumCrankRev - _prevCumulativeCrankRev;
  if (deltaRot < 0)
    deltaRot += 65536;
  int timeDelta = lastCrankTime - _prevCrankTime;
  if (timeDelta < 0)
    timeDelta += 65536;

  if (timeDelta > 0)
  {
    _prevCrankStaleness = 0;
    _prevRPM = ((double)deltaRot) / (((double)timeDelta) / 1024.0 / 60.0);
  }
  else if (_prevCrankStaleness < 2)
    _prevCrankStaleness++;
  else
    _prevRPM = 0.0;

  _prevCumulativeCrankRev = cumCrankRev;
  _prevCrankTime = lastCrankTime;
  _cadence = (_prevRPM > 160) ? 0 : (int)_prevRPM;
}

int CadenceSensor::getCadence() { return _cadence; }
bool CadenceSensor::isConnected() { return _connected; }
bool CadenceSensor::foundNewDevice() { return _newDeviceFound; }
const char *CadenceSensor::getNewMac() { return _newMac.c_str(); }
const char *CadenceSensor::getNewName() { return _newName.c_str(); }
uint8_t CadenceSensor::getNewAddressType() { return _newType; }
void CadenceSensor::clearNewDeviceFlag() { _newDeviceFound = false; }