#ifndef DEVICE_H
#define DEVICE_H

#include <BLEDevice.h>

static BLEUUID serviceUUID("00001816-0000-1000-8000-00805f9b34fb");
static BLEUUID notifyUUID("00002a5b-0000-1000-8000-00805f9b34fb");

static BLEAdvertisedDevice* devices[20];
static int device_count = 0;

void addDevice(BLEAdvertisedDevice* device) {
    for (uint8_t i = 0; i < device_count; i++) {
        if (device->getName() == devices[i]->getName()) return;
    }
    if (!device->haveServiceUUID() || !device->isAdvertisingService(serviceUUID)) return;
    
    devices[device_count] = device;
    device_count++;
}

BLEAdvertisedDevice* selectDevice(void) {
    if (device_count == 0) return nullptr;
    return devices[0]; // Selects the first found device
}

#endif