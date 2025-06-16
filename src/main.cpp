#include "Arduino.h"
#include "BLEDevice.h" // Library for BLE client functionality
#include "device.h"    // Custom header, likely containing UUIDs and device list logic

#include "EEPROM.h"      // Library for non-volatile storage
#define EEPROM_SIZE 128  // Allocate 128 bytes in EEPROM for storing settings

// --- Global State Variables for BLE ---
static boolean connected = false;                            // Tracks the current BLE connection state
static BLERemoteCharacteristic *sensorCharacteristic;      // Pointer to the BLE characteristic we will read from
static BLEAdvertisedDevice *device;                          // Pointer to the BLE device we are connecting to
static BLEClient *client;                                    // Pointer to the BLE client instance
static BLEScan *scanner;                                     // Pointer to the BLE scanner instance
bool SCAN_FOR_DEVICE = false; // Flag to control whether to scan for a new device on startup

// --- Global State Variables for Application Logic ---
static int cadence = 0;                     // Current cadence value in RPM, received from the sensor
static unsigned long runtime = 0;           // Total time in milliseconds that the cadence has been > 0
static unsigned long last_millis = 0;       // Timestamp for calculating runtime

// --- Variables for Cadence Calculation ---
static int prevCumulativeCrankRev = 0;      // Previous crank revolution count from the sensor
static int prevCrankTime = 0;               // Previous crank time measurement from the sensor
static double rpm = 0;                      // Calculated RPM, stored as a double for precision
static double prevRPM = 0;                  // Previous RPM value, used for staleness calculation
static int prevCrankStaleness = 0;          // Counter to track how many updates have been missed
static int stalenessLimit = 2;              // Number of missed updates before setting RPM to 0
static int scanCount = 0;                   // Counter for scan attempts

// --- Device Configuration and Settings ---
struct DeviceInfo
{
  char macAddress[18];        // MAC address of the saved BLE sensor
  char deviceName[20];        // Name of the saved BLE sensor
  esp_ble_addr_type_t addressType; // BLE address type (public, random, etc.)
  bool SCAN_FOR_DEVICE;       // The scan flag, saved to EEPROM
  int brakingThreshold;       // The threshold value for braking detection
  int brakingTimeout;         // The timeout value for braking detection
};

DeviceInfo deviceInfo; // Global instance of the device settings struct

// --- Function Prototypes ---
void saveDeviceToEEPROM(BLEAdvertisedDevice *device);
void overwriteSCAN_FOR_DEVICE(bool value);

// --- Pin Definitions and Configuration ---
#define debug 0           // Set to 1 to enable extra Serial.println statements for debugging
#define maxCadence 160    // Maximum plausible cadence value; readings above this are ignored
int brakingTimeout = 2000;    // Default braking detection timeout in milliseconds
int brakingThreshold = 2048; // Default analog threshold for braking detection

// Pin assignments for hardware
#define throttle 8
#define LED 8             // LED is on the same pin as throttle
#define PASoutput 9       // Pin to output the simulated PAS signal
#define brakeInput 2      // (Unused in logic) Intended for a brake lever switch
#define brakeOutput 10    // Pin that signals braking to the motor controller
#define inductiveProbe 3  // Analog pin for the inductive braking sensor

/**
 * @brief Detects a braking state using a hysteresis algorithm.
 * @details This function is called frequently and uses a counter to filter out sensor noise.
 * It requires a certain number of consecutive readings above/below a threshold to change state,
 * preventing flickering. It is designed to be run at approximately 100Hz.
 *
 * @param timeout The time in milliseconds to wait before considering braking as detected.
 *                This is converted to a counter threshold based on the 10ms tick rate.
 * @return true if braking is detected, false otherwise.
 */
bool detectBraking(uint16_t timeout)
{
  // Convert timeout from milliseconds to a sample counter threshold.
  // The calculation assumes the function is ticked every 10ms.
  // The division by 2 is an additional, specific tuning factor for this implementation.
  timeout = timeout / 2 / 10;

  // Static variables to persist state between function calls
  static uint32_t prevMillis = 0;      // Timestamp of the last execution
  static int16_t brakingCounter = 0;   // The hysteresis counter
  static bool brakingDetected = false; // The current confirmed braking state

  uint32_t currentMillis = millis();

  // Run the logic at a fixed rate of approximately 100Hz (every 10ms)
  if (currentMillis - prevMillis > 10)
  {
    prevMillis = currentMillis;

    // Check if the sensor reading is below the braking threshold
    if (analogRead(inductiveProbe) < 2000)
    {
      brakingCounter++; // Increment counter towards the 'braking' state
      // If the counter exceeds the positive threshold, confirm braking is active
      if (brakingCounter > timeout)
      {
        brakingCounter = timeout; // Clamp the counter to the threshold to lock the state
        brakingDetected = true;
      }
    }
    else // Sensor reading is above the threshold
    {
      brakingCounter--; // Decrement counter towards the 'not braking' state
      // If the counter falls below the negative threshold, confirm braking is inactive
      if (brakingCounter < -timeout)
      {
        brakingCounter = -timeout; // Clamp the counter to the negative threshold
        brakingDetected = false;
      }
    }
  }
  return brakingDetected; // Return the current confirmed state
}

/**
 * @brief Generates a square wave on the PASoutput pin to simulate a PAS sensor.
 * @details The frequency of the square wave is proportional to the input cadence.
 *
 * @param cadenceIn The current cadence in RPM.
 */
void bitBangPAS(float cadenceIn)
{
  // Convert RPM to pulses per minute, assuming 12 magnets on the crank
  cadenceIn = cadenceIn * 12;
  // Calculate the signal frequency in Hertz
  float hertz = cadenceIn / 60;
  // Calculate the half-period in milliseconds for a 50% duty cycle square wave
  uint16_t period = 1000.0 / hertz;
  period = period / 2;

  // Static variables to maintain state for the non-blocking toggle
  static uint32_t prevMillis = millis();
  static bool state = false;

  uint32_t currentMillis = millis();
  // Use a non-blocking delay to toggle the output pin at the calculated period
  if (currentMillis - prevMillis > period)
  {
    digitalWrite(PASoutput, state);
    prevMillis = currentMillis;
    state = !state; // Invert the state for the next toggle
  }
}

// ----------------- ESPUI Web Interface Section -----------------
#include <DNSServer.h>
#include <ESPUI.h>

const byte DNS_PORT = 53;         // Standard DNS port
IPAddress apIP(192, 168, 4, 1);   // IP address for the ESP32 Access Point
DNSServer dnsServer;              // DNS server to handle captive portal requests

#include <WiFi.h>

const char *hostname = "espui";      // Hostname for the ESP32 on the network
const char *ssid = "ESP-Cadence";    // SSID of the WiFi Access Point
const char *password = "12345678";   // Password for the WiFi Access Point

// IDs for ESPUI controls to reference them in callbacks and updates
int labelID1, labelID2, labelID3, labelID4, labelID5, labelID6, labelID7, labelID8;

/**
 * @brief Callback function for handling number input changes from the UI.
 */
void numberCall(Control *sender, int type)
{
  if (sender->id == labelID1) // Brake Threshold
  {
    brakingThreshold = sender->value.toInt();
    deviceInfo.brakingThreshold = brakingThreshold;
  }
  else if (sender->id == labelID7) // Brake Timeout
  {
    brakingTimeout = sender->value.toInt();
    deviceInfo.brakingTimeout = brakingTimeout;
  }
}

/**
 * @brief Callback function for handling button presses from the UI.
 */
void buttonCallback(Control *sender, int type)
{
  if (sender->id == labelID6) // "Save Values to EEPROM" button
  {
    Serial.println("Button 'Save' pressed: saving values to EEPROM.");
    overwriteSCAN_FOR_DEVICE(false); // Saves all current settings
  }
  else if (sender->id == labelID8) // "Search for Cadence Sensor" button
  {
    Serial.println("Button 'Search' pressed: activating scan mode.");
    overwriteSCAN_FOR_DEVICE(true); // Set the scan flag in EEPROM
    SCAN_FOR_DEVICE = true;         // Set the live flag for the current session
  }
}

/**
 * @brief Initializes the ESP32 in Access Point (AP) mode and sets up the ESPUI web interface.
 */
void setupWiFi()
{
  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);

  // Start DNS server for captive portal functionality (redirects all traffic to the ESP's IP)
  dnsServer.start(DNS_PORT, "*", apIP);

  // Create all the UI elements for the web page
  labelID1 = ESPUI.number("Brake Threshold", &numberCall, ControlColor::Alizarin, brakingThreshold, 0, 4096);
  labelID2 = ESPUI.label("Cadence", ControlColor::Emerald, "0");
  labelID3 = ESPUI.label("analogValue", ControlColor::Emerald, "0");
  labelID4 = ESPUI.graph("cadence", ControlColor::Wetasphalt);
  labelID5 = ESPUI.graph("analogValue", ControlColor::Wetasphalt);
  labelID7 = ESPUI.number("Brake Timeout", &numberCall, ControlColor::Alizarin, brakingTimeout, 0, 10000);
  labelID6 = ESPUI.button("Save Values to EEPROM", &buttonCallback, ControlColor::Peterriver, "Save");
  labelID8 = ESPUI.button("Activate Search for Cadence Sensor", &buttonCallback, ControlColor::Peterriver, "Search");

  // Start the ESPUI web server
  ESPUI.begin("ESPUI Control");
}

/**
 * @brief Maintains the ESPUI web server and updates the UI elements periodically.
 */
void ESPuiRun()
{
  // Process any DNS requests for the captive portal
  dnsServer.processNextRequest();

  static unsigned long oldTime = 0;
  static unsigned long lastPurge = 0;
  unsigned long currentTime = millis();

  // Update labels and graph points every 500ms
  if (currentTime - oldTime > 500)
  {
    ESPUI.print(labelID2, String(cadence));
    ESPUI.print(labelID3, String(analogRead(inductiveProbe))); // Read probe directly for UI
    ESPUI.addGraphPoint(labelID4, cadence);
    ESPUI.addGraphPoint(labelID5, analogRead(inductiveProbe));
    oldTime = currentTime;
  }

  // Clear graph data every 10 seconds to prevent it from getting too cluttered
  if (currentTime - lastPurge > 10000)
  {
    ESPUI.clearGraph(labelID4);
    ESPUI.clearGraph(labelID5);
    lastPurge = currentTime;
  }
}

// ----------------- EEPROM Storage Section -----------------

// Helper function to check a specific bit in a value
static bool is_bit_set(unsigned value, unsigned bitindex)
{
  return (value & (1 << bitindex)) != 0;
}

// EEPROM address where the DeviceInfo struct will be stored
constexpr int EEPROM_ADDRESS = 0;

/**
 * @brief Saves the connected BLE device's info and current settings into the DeviceInfo struct
 *        and writes it to EEPROM for persistence across reboots.
 * @param device Pointer to the BLE device that was connected.
 */
void saveDeviceToEEPROM(BLEAdvertisedDevice *device)
{
  String macAddress = device->getAddress().toString().c_str();
  String deviceName = device->getName().c_str();

  // Copy data into the global deviceInfo struct, ensuring null termination
  strncpy(deviceInfo.macAddress, macAddress.c_str(), sizeof(deviceInfo.macAddress) - 1);
  deviceInfo.macAddress[sizeof(deviceInfo.macAddress) - 1] = '\0';
  strncpy(deviceInfo.deviceName, deviceName.c_str(), sizeof(deviceInfo.deviceName) - 1);
  deviceInfo.deviceName[sizeof(deviceInfo.deviceName) - 1] = '\0';
  deviceInfo.addressType = device->getAddressType();
  deviceInfo.SCAN_FOR_DEVICE = SCAN_FOR_DEVICE;
  // Braking values are updated directly in their callbacks

  // Write the entire struct to EEPROM
  EEPROM.put(EEPROM_ADDRESS, deviceInfo);
  EEPROM.commit(); // Ensure data is written to flash
}

/**
 * @brief Reads the DeviceInfo struct from EEPROM.
 * @details Validates if a device was previously saved by checking the MAC address length.
 *
 * @param deviceInfo Reference to the struct where the loaded data will be stored.
 * @return true if valid data was loaded, false otherwise.
 */
bool retrieveDeviceFromEEPROM(DeviceInfo &deviceInfo)
{
  EEPROM.get(EEPROM_ADDRESS, deviceInfo);
  // A simple validation check: a valid MAC address string is 17 characters long.
  if (strlen(deviceInfo.macAddress) == 17)
  {
    return true;
  }
  return false;
}

/**
 * @brief Specifically updates the 'SCAN_FOR_DEVICE' flag and all other current settings in EEPROM.
 * @param value The new boolean value for the scan flag.
 */
void overwriteSCAN_FOR_DEVICE(bool value)
{
  deviceInfo.SCAN_FOR_DEVICE = value;
  // This function effectively acts as a general "save all settings"
  EEPROM.put(EEPROM_ADDRESS, deviceInfo);
  EEPROM.commit();
}

// ----------------- Serial Command Handler Section -----------------

/**
 * @brief Checks for incoming commands from the Serial monitor to trigger actions.
 */
void handleSerialCommands()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove whitespace

    if (command.equalsIgnoreCase("scan"))
    {
      Serial.println("Will reboot and start scan for devices...");
      overwriteSCAN_FOR_DEVICE(true); // Set flag to scan on next boot
      delay(1000);                    // Wait for Serial to finish printing
      ESP.restart();
    }
    else
    {
      Serial.println("Unknown command. Available commands:");
      Serial.println("  scan - Start scanning for BLE devices on next boot");
    }
  }
}

// ----------------- BLE Client Logic Section -----------------

/**
 * @brief Callback function executed when the BLE sensor sends a data notification.
 * @details This is the core of the BLE data processing. It parses the incoming byte array
 *          to calculate the cadence (RPM).
 */
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *data, size_t length, bool isNotify)
{
  // The first byte of the payload is a flags field.
  bool hasWheel = is_bit_set(data[0], 0); // Check if wheel revolution data is present
  bool hasCrank = is_bit_set(data[0], 1); // Check if crank revolution data is present

  // The payload structure changes depending on whether wheel data is present.
  // We only care about crank data.
  int crankRevIndex = 1;
  int crankTimeIndex = 3;
  if (hasWheel)
  {
    // If wheel data is present, crank data is offset further in the payload.
    crankRevIndex = 7;
    crankTimeIndex = 9;
  }

  // Extract cumulative crank revolutions and last crank event time from the payload.
  // These are 16-bit values, constructed from two bytes (little-endian).
  int cumulativeCrankRev = int((data[crankRevIndex + 1] << 8) | data[crankRevIndex]);
  int lastCrankTime = int((data[crankTimeIndex + 1] << 8) | data[crankTimeIndex]);

  // Calculate the change in revolutions and time since the last update.
  int deltaRotations = cumulativeCrankRev - prevCumulativeCrankRev;
  // Handle 16-bit counter rollover (e.g., from 65535 to 0).
  if (deltaRotations < 0)
  {
    deltaRotations += 65535;
  }

  int timeDelta = lastCrankTime - prevCrankTime;
  // Handle 16-bit counter rollover for time.
  if (timeDelta < 0)
  {
    timeDelta += 65535;
  }

  // Calculate RPM based on the deltas.
  if (timeDelta != 0) // A new event occurred
  {
    prevCrankStaleness = 0; // Reset staleness counter
    // Time is in units of 1/1024 seconds. Convert to minutes.
    double timeMins = ((double)timeDelta) / 1024.0 / 60.0;
    rpm = ((double)deltaRotations) / timeMins;
    prevRPM = rpm; // Store this RPM in case the next update is stale
  }
  else if (timeDelta == 0 && prevCrankStaleness < stalenessLimit)
  {
    // No new event, but we haven't hit the staleness limit yet.
    // Assume the same RPM as the last valid measurement to avoid dropping to 0 briefly.
    rpm = prevRPM;
    prevCrankStaleness++;
  }
  else // No new event and the staleness limit has been reached.
  {
    rpm = 0.0; // Assume pedaling has stopped.
  }

  // Update previous values for the next calculation
  prevCumulativeCrankRev = cumulativeCrankRev;
  prevCrankTime = lastCrankTime;

  // Final cadence value, capped at maxCadence
  cadence = (int)rpm;
  if (cadence > maxCadence)
  {
    cadence = 0; // Treat impossibly high values as an error and reset
  }
}

/**
 * @brief Callback class for handling BLE client connection and disconnection events.
 */
class ClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
    Serial.println("Connected!");
    // If we just scanned for this device, save its info to EEPROM for future connections.
    if (SCAN_FOR_DEVICE)
    {
      SCAN_FOR_DEVICE = false;
      saveDeviceToEEPROM(device);
    }
  }

  void onDisconnect(BLEClient *pclient)
  {
    connected = false;
    delete client; // Clean up the client object
    client = nullptr;
    Serial.println("Disconnected!");
  }
};

/**
 * @brief Callback class for handling the results of a BLE scan.
 */
class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  // Called for each BLE device found during a scan.
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    // The `device.h` header is expected to contain the `addDevice` function
    // which adds the found device to a list for later selection.
    if (advertisedDevice.getName().size() > 0)
    {
      BLEAdvertisedDevice *d = new BLEAdvertisedDevice;
      *d = advertisedDevice;
      addDevice(d);
    }
  }
};

/**
 * @brief Attempts to establish a BLE connection to the target sensor.
 * @return true on successful connection and characteristic registration, false otherwise.
 */
bool connectToServer()
{
  client = BLEDevice::createClient();
  client->setClientCallbacks(new ClientCallback());

  bool result = false;
  // If in scan mode, connect to the device just found.
  if (SCAN_FOR_DEVICE)
  {
    Serial.print("Connecting to ");
    Serial.println(device->getName().c_str());
    result = client->connect(device);
  }
  // Otherwise, connect to the device saved in EEPROM.
  else
  {
    Serial.print("Connecting to ");
    Serial.println(deviceInfo.deviceName);
    BLEAddress bleAddress(deviceInfo.macAddress);
    result = client->connect(bleAddress, deviceInfo.addressType);
  }

  delay(200);
  if (!result)
  {
    return false;
  }

  // After connecting, get the service we are interested in.
  // serviceUUID is defined in device.h
  BLERemoteService *remoteService = client->getService(serviceUUID);
  if (remoteService == nullptr)
  {
    client->disconnect();
    return false;
  }

  // Get the characteristic we want to read notifications from.
  // notifyUUID is defined in device.h
  sensorCharacteristic = remoteService->getCharacteristic(notifyUUID);
  if (sensorCharacteristic == nullptr)
  {
    client->disconnect();
    return false;
  }

  // Register our callback function to receive notifications.
  sensorCharacteristic->registerForNotify(notifyCallback);

  return true;
}

// ----------------- Main Setup and Loop -----------------

/**
 * @brief Standard Arduino setup function. Runs once on startup.
 */
void setup()
{
  Serial.begin(460800);
  delay(50);

  // Initialize EEPROM
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialize EEPROM");
    while(1); // Halt on failure
  }

  // Load saved settings from EEPROM if not in forced scan mode.
  if (!SCAN_FOR_DEVICE)
  {
    if (retrieveDeviceFromEEPROM(deviceInfo))
    {
      SCAN_FOR_DEVICE = deviceInfo.SCAN_FOR_DEVICE;
      brakingThreshold = deviceInfo.brakingThreshold;
      brakingTimeout = deviceInfo.brakingTimeout;
    }
  }

  // Initialize the WiFi AP and Web UI
  setupWiFi();

  // Configure hardware pins
  pinMode(throttle, OUTPUT);
  digitalWrite(throttle, LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // LED HIGH means off (common anode) or just default state
  pinMode(PASoutput, OUTPUT);
  digitalWrite(PASoutput, LOW);
  pinMode(brakeOutput, INPUT); // Default to input (high-Z) to not interfere
  pinMode(inductiveProbe, INPUT_PULLUP);

  // Initialize the BLE stack
  BLEDevice::init("");
  scanner = BLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  scanner->setInterval(1349); // BLE scan interval
  scanner->setWindow(449);    // BLE scan window
  scanner->setActiveScan(true);
}

/**
 * @brief Main application loop.
 */
void loop()
{
  // Keep the web server and serial command handler running
  ESPuiRun();
  handleSerialCommands();

  // --- Main State Machine: Scan or Connect ---
  if (SCAN_FOR_DEVICE)
  {
    // STATE: SCAN MODE. If we are not connected, start a scan.
    if (!connected)
    {
      scanner->start(11, false); // Scan for 11 seconds
      BLEDevice::getScan()->stop();

      device = selectDevice(); // selectDevice() is expected to be in device.h
      if (device != nullptr)
      {
        connected = connectToServer();
        if (!connected) return; // Retry on next loop iteration
        scanCount = 0;
      }
      else
      {
        Serial.println("No compatible device found...");
        scanCount++;
        if (scanCount > 5) { /* Optional: sleep or restart after too many failed scans */ }
        delay(3100);
        return;
      }
    }
  }
  else // NOT in SCAN_FOR_DEVICE mode
  {
    // STATE: CONNECT MODE. If not connected, try to connect to the saved device.
    if (!connected)
    {
      connected = connectToServer();
      if (!connected) return; // Retry on next loop
    }
  }

  // --- Runtime Calculation ---
  if (cadence > 0)
  {
    unsigned long now = millis();
    runtime += now - last_millis;
    last_millis = now;
  }
  else
  {
    last_millis = millis(); // Keep last_millis updated when stopped
  }

  // --- Core Operational Logic ---
  // Braking detection has the highest priority.
  if (detectBraking(brakingTimeout))
  {
    // If braking is detected, override all motor outputs.
    pinMode(brakeOutput, OUTPUT);   // Set pin to OUTPUT to actively signal
    digitalWrite(brakeOutput, LOW); // Signal braking (assuming active LOW)
    digitalWrite(PASoutput, LOW);   // Stop PAS signal
    digitalWrite(LED, HIGH);        // Turn on indicator LED (assuming active LOW)
  }
  else
  {
    // If not braking, revert brakeOutput to a high-impedance state.
    pinMode(brakeOutput, INPUT);

    if (cadence > 0)
    {
      // If moving, generate PAS signal and turn off indicator LED.
      digitalWrite(LED, LOW);
      bitBangPAS(cadence);
    }
    else
    {
      // If stopped, ensure all motor-related outputs are off.
      digitalWrite(PASoutput, LOW);
      digitalWrite(LED, HIGH);
    }
  }

  // --- Serial Logging ---
  // Print cadence to Serial only when the value changes to reduce console spam.
  static int prevCadence = 0;
  if (cadence != prevCadence)
  {
    Serial.print("Cadence: ");
    Serial.println(cadence);
    prevCadence = cadence;
  }
}