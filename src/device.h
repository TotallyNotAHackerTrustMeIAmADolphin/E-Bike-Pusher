#ifndef DEVICE_H
#define DEVICE_H

#include "BLEDevice.h"

// Standard speed & cadence Bluetooth UUID
static BLEUUID serviceUUID("00001816-0000-1000-8000-00805f9b34fb");
// Standard notify characteristic UUID
static BLEUUID notifyUUID("00002a5b-0000-1000-8000-00805f9b34fb");

static BLEAdvertisedDevice* devices[20];
static int device_count = 0;

/**
 * @brief Add a device to the device list, avoiding duplicates.
 * 
 * @param device Pointer to the BLEAdvertisedDevice to be added.
 */
void addDevice(BLEAdvertisedDevice* device) {
    for (uint8_t i = 0; i < device_count; i++) {
        if (device->getName() == devices[i]->getName()) {
            // Device already in the list, no need to add again
            return;
        }
    }
    if (!device->haveServiceUUID() || !device->isAdvertisingService(serviceUUID)) {
        // Device does not advertise the required service UUID
        return;
    }
    devices[device_count] = device;
    device_count++;
}

/**
 * @brief Select a device from the list.
 * 
 * @return BLEAdvertisedDevice* Pointer to the selected BLEAdvertisedDevice.
 */
BLEAdvertisedDevice* selectDevice(void) {
    if (device_count == 0)
        return nullptr; // No devices found
    if (device_count == 1)
        return devices[0]; // Only one device, select it

    int selected = 0;

    // If multiple devices, return the first one (can be enhanced)
    return devices[selected];
}

#endif // DEVICE_H
