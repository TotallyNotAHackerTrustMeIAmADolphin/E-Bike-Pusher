#ifndef CADENCE_SENSOR_H
#define CADENCE_SENSOR_H

#include <Arduino.h>
#include <NimBLEDevice.h> // <--- NEW LIBRARY

extern void addLog(const char* msg);

class CadenceSensor {
  public:
    CadenceSensor();
    void begin(bool scanMode, const char* savedMac, esp_ble_addr_type_t savedType);
    void loop();
    
    int getCadence();
    bool isConnected();
    
    bool foundNewDevice();
    const char* getNewMac();
    const char* getNewName();
    esp_ble_addr_type_t getNewAddressType();
    void clearNewDeviceFlag();

    void _onNotify(uint8_t* data, size_t length);
    void _onConnect();
    void _onDisconnect();
    void _onScanResult(NimBLEAdvertisedDevice* advertisedDevice);
    void _onScanComplete();

  private:
    bool _scanMode;
    bool _connected;
    bool _newDeviceFound;
    bool _isScanning; // <--- Prevents blocking!
    
    String _targetMac;
    esp_ble_addr_type_t _targetType;
    String _newName;
    String _newMac;
    esp_ble_addr_type_t _newType;

    NimBLEClient* _client;
    NimBLEScan* _scanner;
    NimBLEAdvertisedDevice* _foundDevice;
    
    int _cadence;
    unsigned long _lastEventTime;
    int _prevCumulativeCrankRev;
    int _prevCrankTime;
    int _prevCrankStaleness;
    double _prevRPM;

    bool connectToServer();
};
#endif