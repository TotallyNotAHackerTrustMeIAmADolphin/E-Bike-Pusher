#ifndef CADENCE_SENSOR_H
#define CADENCE_SENSOR_H

#include <Arduino.h>
#include <NimBLEDevice.h>

extern void addLog(const char *msg);

class CadenceSensor
{
public:
  CadenceSensor();
  void begin(bool scanMode, const char *savedMac, uint8_t savedType);
  void loop();

  int getCadence();
  bool isConnected();

  bool foundNewDevice();
  const char *getNewMac();
  const char *getNewName();
  uint8_t getNewAddressType();
  void clearNewDeviceFlag();

  void _onNotify(uint8_t *data, size_t length);
  void _onConnect();
  void _onDisconnect();
  void _onScanResult(NimBLEAdvertisedDevice *advertisedDevice);
  void _onScanComplete();

private:
  bool _scanMode;
  bool _connected;
  bool _newDeviceFound;
  bool _isScanning;
  bool _scanFinished; // <--- NEW: Thread safety flag
  bool _isConnecting; // <--- NEW: Thread safety flag

  String _targetMac;
  uint8_t _targetType;
  String _newName;
  String _newMac;
  uint8_t _newType;

  NimBLEClient *_client;
  NimBLEScan *_scanner;
  NimBLEAdvertisedDevice *_foundDevice;

  int _cadence;
  unsigned long _lastEventTime;
  int _prevCumulativeCrankRev;
  int _prevCrankTime;
  int _prevCrankStaleness;
  double _prevRPM;

  bool connectToServer();
};
#endif