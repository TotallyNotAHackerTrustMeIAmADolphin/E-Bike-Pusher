#include "Arduino.h"
#include "BLEDevice.h"
#include "device.h"

#include "EEPROM.h"
#define EEPROM_SIZE 128

static boolean connected = false;
static BLERemoteCharacteristic *sensorCharacteristic;
static BLEAdvertisedDevice *device;
static BLEClient *client;
static BLEScan *scanner;
bool SCAN_FOR_DEVICE = false; // initialize as true to scan for device

static int cadence = 0;
static unsigned long runtime = 0;
static unsigned long last_millis = 0;

static int prevCumulativeCrankRev = 0;
static int prevCrankTime = 0;
static double rpm = 0;
static double prevRPM = 0;
static int prevCrankStaleness = 0;
static int stalenessLimit = 2; // Reduced from 4 to 2
static int scanCount = 0;

bool connection = false;

struct DeviceInfo
{
  char macAddress[18];
  char deviceName[20];
  esp_ble_addr_type_t addressType;
  bool SCAN_FOR_DEVICE;
  int brakingThreshold;
  int brakingTimeout;
};

DeviceInfo deviceInfo;
void saveDeviceToEEPROM(BLEAdvertisedDevice *device);
void overwriteSCAN_FOR_DEVICE(bool value);

#define debug 0
#define maxCadence 160
int brakingTimeout = 2000;
int brakingThreshold = 2048;

#define throttle 8
#define LED 8
#define PASoutput 9
#define brakeInput 2
#define brakeOutput 10
#define inductiveProbe 3

bool detectBraking(uint16_t timeout)
{
  timeout = timeout/2/10;
  static uint32_t prevMillis = 0;
  static uint32_t timestamp = 0;
  static int16_t brakingCounter = 0;
  static bool brakingDetected = false;
  uint32_t currentMillis = millis();
  // histeresis function for breaking detection run at 100hz
  if (currentMillis - prevMillis > 10)
  {
    prevMillis = currentMillis;
    if (analogRead(inductiveProbe) < 2000)
    {
      brakingCounter++;
      //limit the braking counter to 10
      if(brakingCounter > timeout){
        brakingCounter = 10;
        brakingDetected = true;
      }
    }
    else
    {
      brakingCounter--;
      //limit braking counter to -10
      if (brakingCounter < -timeout)
      {
        brakingCounter = -10;
        brakingDetected = false;
      }
    }
  }
  return brakingDetected;
}

  void bitBangPAS(float cadenceIn)
  {
    cadenceIn = cadenceIn * 12; // times 12 because of 12 pas magnets
    float hertz = cadenceIn / 60;
    uint16_t period = 1000.0 / hertz;
    period = period / 2;
    static uint32_t prevMillis = millis();
    static bool state = false;
    uint32_t currentMillis = millis();
    if (currentMillis - prevMillis > period)
    {
      digitalWrite(PASoutput, state);
      prevMillis = currentMillis;
      state = !state;
    }
  }

// ----------------- ESPUI Stuff -----------------
#include <DNSServer.h>
#include <ESPUI.h>

  const byte DNS_PORT = 53;
  IPAddress apIP(192, 168, 4, 1);
  DNSServer dnsServer;

#include <WiFi.h>

  const char *hostname = "espui";
  const char *ssid = "ESP-Cadence";
  const char *password = "12345678";

  int labelID1;
  int labelID2;
  int labelID3;
  int labelID4;
  int labelID5;
  int labelID6;
  int labelID7;
  int labelID8;

  void numberCall(Control * sender, int type)
  {
    Serial.println(sender->value);
    if (sender->id == labelID1)
    {
      brakingThreshold = sender->value.toInt();
      deviceInfo.brakingThreshold = brakingThreshold;
      Serial.println(brakingThreshold);
    }
    else if (sender->id == labelID7)
    {
      brakingTimeout = sender->value.toInt();
      deviceInfo.brakingTimeout = brakingTimeout;
      Serial.println(brakingTimeout);
    }
  }

  void buttonCallback(Control * sender, int type)
  {
    if (sender->id == labelID6)
    {
      Serial.println("Button pressed");
      overwriteSCAN_FOR_DEVICE(false);
    }
    else if (sender->id == labelID8)
    {
      Serial.println("Button pressed");
      overwriteSCAN_FOR_DEVICE(true);
      SCAN_FOR_DEVICE = true;
    }
  }

  void setupWiFi()
  {
    WiFi.setHostname(hostname);
    WiFi.mode(WIFI_AP);
    delay(100);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(ssid, password);

    int timeout = 5;

    do
    {
      delay(500);
      Serial.print(".");
      timeout--;
    } while (timeout);

    dnsServer.start(DNS_PORT, "*", apIP);

    labelID1 = ESPUI.number("Brake Threshold", &numberCall, ControlColor::Alizarin, brakingThreshold, 0, 4096);
    labelID2 = ESPUI.label("Cadence", ControlColor::Emerald, "0");
    labelID3 = ESPUI.label("analogValue", ControlColor::Emerald, "0");
    labelID4 = ESPUI.graph("cadence", ControlColor::Wetasphalt);
    labelID5 = ESPUI.graph("analogValue", ControlColor::Wetasphalt);
    labelID7 = ESPUI.number("Brake Timeout", &numberCall, ControlColor::Alizarin, brakingTimeout, 0, 10000);
    labelID6 = ESPUI.button("Save Values to EEPROM", &buttonCallback, ControlColor::Peterriver, "Save");
    labelID8 = ESPUI.button("Activate Search for Cadence Sensor", &buttonCallback, ControlColor::Peterriver, "Search");

    ESPUI.begin("ESPUI Control");
  }

  void ESPuiRun()
  {
    dnsServer.processNextRequest();

    static unsigned long oldTime = 0;
    static unsigned long lastPurge = 0;

    unsigned long currentTime = millis();

    if (currentTime - oldTime > 500)
    {
      ESPUI.print(labelID2, String(cadence));
      ESPUI.print(labelID3, String(analogRead(brakeInput)));

      ESPUI.addGraphPoint(labelID4, cadence);
      ESPUI.addGraphPoint(labelID5, analogRead(brakeInput));

      oldTime = currentTime;
    }
    if (currentTime - lastPurge > 10000)
    {
      ESPUI.clearGraph(labelID4);
      ESPUI.clearGraph(labelID5);
      lastPurge = currentTime;
    }
  }

  // -----------------------------------------------

  static bool is_bit_set(unsigned value, unsigned bitindex)
  {
    return (value & (1 << bitindex)) != 0;
  }

  constexpr int EEPROM_ADDRESS = 0;

  void saveDeviceToEEPROM(BLEAdvertisedDevice * device)
  {
    String macAddress = device->getAddress().toString().c_str();
    String deviceName = device->getName().c_str();

    strncpy(deviceInfo.macAddress, macAddress.c_str(), sizeof(deviceInfo.macAddress));
    strncpy(deviceInfo.deviceName, deviceName.c_str(), sizeof(deviceInfo.deviceName));
    deviceInfo.addressType = device->getAddressType();
    deviceInfo.SCAN_FOR_DEVICE = SCAN_FOR_DEVICE;
    // deviceInfo.brakingThreshold = brakingThreshold;

    EEPROM.put(EEPROM_ADDRESS, deviceInfo);
    EEPROM.commit();
  }

  bool retrieveDeviceFromEEPROM(DeviceInfo & deviceInfo)
  {
    EEPROM.get(EEPROM_ADDRESS, deviceInfo);
    if (strlen(deviceInfo.macAddress) == 17)
    { // Validate MAC address length
      return true;
    }
    return false;
  }

  void overwriteSCAN_FOR_DEVICE(bool value)
  {
    deviceInfo.SCAN_FOR_DEVICE = value;
    EEPROM.put(EEPROM_ADDRESS, deviceInfo);
    EEPROM.commit();
  }

  void handleSerialCommands()
  {
    if (Serial.available())
    {
      String command = Serial.readStringUntil('\n');
      command.trim(); // Remove any trailing newline or spaces

      if (command.equalsIgnoreCase("scan"))
      {
        Serial.println("Will reboot and start scan for devices...");
        overwriteSCAN_FOR_DEVICE(true);
        delay(1000);
        ESP.restart();
      }
      else
      {
        Serial.println("Unknown command. Available commands:");
        Serial.println("  scan - Start scanning for BLE devices");
      }
    }
  }

  // Called when device sends update notification
  static void notifyCallback(BLERemoteCharacteristic * pBLERemoteCharacteristic, uint8_t *data, size_t length, bool isNotify)
  {

    bool hasWheel = is_bit_set(data[0], 0);
    bool hasCrank = is_bit_set(data[0], 1);

    int crankRevIndex = 1;
    int crankTimeIndex = 3;
    if (hasWheel)
    {
      crankRevIndex = 7;
      crankTimeIndex = 9;
    }

    int cumulativeCrankRev = int((data[crankRevIndex + 1] << 8) + data[crankRevIndex]);
    int lastCrankTime = int((data[crankTimeIndex + 1] << 8) + data[crankTimeIndex]);

    if (debug)
    {
      Serial.println("Notify callback for characteristic");
      Serial.print("cumulativeCrankRev: ");
      Serial.println(cumulativeCrankRev);
      Serial.print("lastCrankTime: ");
      Serial.println(lastCrankTime);
    }

    int deltaRotations = cumulativeCrankRev - prevCumulativeCrankRev;
    if (deltaRotations < 0)
    {
      deltaRotations += 65535;
    }

    int timeDelta = lastCrankTime - prevCrankTime;
    if (timeDelta < 0)
    {
      timeDelta += 65535;
    }

    if (debug)
    {
      Serial.print("deltaRotations: ");
      Serial.println(deltaRotations);
      Serial.print("timeDelta: ");
      Serial.println(timeDelta);
    }

    // If new data is received, reset staleness and update RPM
    if (timeDelta != 0)
    {
      prevCrankStaleness = 0;
      double timeMins = ((double)timeDelta) / 1024.0 / 60.0;
      rpm = ((double)deltaRotations) / timeMins;
      prevRPM = rpm;

      if (debug)
      {
        Serial.print("timeMins: ");
        Serial.println(timeMins);
        Serial.print("timeDelta != 0: rpm - ");
        Serial.println(rpm);
      }
    }
    else if (timeDelta == 0 && prevCrankStaleness < stalenessLimit)
    {
      rpm = prevRPM;
      prevCrankStaleness += 1;

      if (debug)
      {
        Serial.print("timeDelta == 0 and not stale yet, rpm - ");
        Serial.println(rpm);
      }
    }
    else if (prevCrankStaleness >= stalenessLimit)
    {
      rpm = 0.0;

      if (debug)
      {
        Serial.print("stale ");
        Serial.println(rpm);
      }
    }

    prevCumulativeCrankRev = cumulativeCrankRev;
    prevCrankTime = lastCrankTime;

    if (debug)
    {
      Serial.print("prevCumulativeCrankRev: ");
      Serial.println(prevCumulativeCrankRev);
      Serial.print("prevCrankTime: ");
      Serial.println(prevCrankTime);
    }

    cadence = (int)rpm;
    if (cadence > maxCadence)
    {
      cadence = 0;
    }

    if (debug)
    {
      Serial.print("CALLBACK(");
      Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
      Serial.print(":");
      Serial.print(length);
      Serial.print("):");
      for (int x = 0; x < length; x++)
      {
        if (data[x] < 16)
        {
          Serial.print("0");
        }
        Serial.print(data[x], HEX);
      }
      Serial.println();
    }
  }

  // Called on connect or disconnect
  class ClientCallback : public BLEClientCallbacks
  {
    void onConnect(BLEClient *pclient)
    {
      Serial.println("Connected!");
      if (SCAN_FOR_DEVICE)
      {
        SCAN_FOR_DEVICE = false;
        saveDeviceToEEPROM(device);
      }
    }
    void onDisconnect(BLEClient *pclient)
    {
      connected = false;
      delete client;
      client = nullptr;
      Serial.println("Disconnected!");
    }
  };

  class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
  {
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
      Serial.print("BLE Advertised Device found: ");
      Serial.print(advertisedDevice.getAddressType());
      Serial.print("   ");
      Serial.println(advertisedDevice.toString().c_str());
      if (advertisedDevice.getName().size() > 0)
      {
        BLEAdvertisedDevice *d = new BLEAdvertisedDevice;
        *d = advertisedDevice;
        addDevice(d);
      }
    }
  };

  bool connectToServer()
  {
    bool result = false;
    client = BLEDevice::createClient();
    client->setClientCallbacks(new ClientCallback());
    if (SCAN_FOR_DEVICE)
    {
      Serial.print("Connecting to ");
      Serial.println(device->getName().c_str());
      result = client->connect(device);
    }
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

    BLERemoteService *remoteService = client->getService(serviceUUID);
    if (remoteService == nullptr)
    {
      Serial.print("Failed to find service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      client->disconnect();
      return false;
    }
    Serial.println("Found device.");

    sensorCharacteristic = remoteService->getCharacteristic(notifyUUID);
    if (sensorCharacteristic == nullptr)
    {
      Serial.print("Failed to find sensor characteristic UUID: ");
      Serial.println(notifyUUID.toString().c_str());
      client->disconnect();
      return false;
    }
    sensorCharacteristic->registerForNotify(notifyCallback);
    Serial.println("Enabled sensor notifications.");
    Serial.println("Activated status callbacks.");

    return true;
  }

  void setup()
  {
    // Serial
    Serial.begin(460800);
    delay(50);

    Serial.println("point2");

    if (!EEPROM.begin(EEPROM_SIZE))
    {
      Serial.println("failed to initialize EEPROM");
      delay(1000000);
    }

    if (!SCAN_FOR_DEVICE)
    {
      if (retrieveDeviceFromEEPROM(deviceInfo))
      {
        SCAN_FOR_DEVICE = deviceInfo.SCAN_FOR_DEVICE;
        brakingThreshold = deviceInfo.brakingThreshold;
        brakingTimeout = deviceInfo.brakingTimeout;
      }
    }

    setupWiFi();

    pinMode(throttle, OUTPUT);
    digitalWrite(throttle, LOW);
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    pinMode(PASoutput, OUTPUT);
    digitalWrite(PASoutput, LOW);
    pinMode(brakeOutput, INPUT);
    pinMode(inductiveProbe, INPUT_PULLUP);

    BLEDevice::init("");
    scanner = BLEDevice::getScan();
    scanner->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
    scanner->setInterval(1349);
    scanner->setWindow(449);
    scanner->setActiveScan(true);

    Serial.println("point2");
  }

  void loop()
  {
    ESPuiRun();

    handleSerialCommands(); // Check for serial commands

    if (SCAN_FOR_DEVICE)
    {
      if (!connected)
      {
        // Serial.println("Start Scan!");
        scanner->start(11, false); // Scan for 10 seconds
        BLEDevice::getScan()->stop();

        device = selectDevice(); // Pick a device
        if (device != nullptr)
        {
          connected = connectToServer();
          if (!connected)
          {
            // Serial.println("Failed to connect...");
            // delay(3100);
            return;
          }
          scanCount = 0;
        }
        else
        {
          Serial.println("No device found...");
          scanCount++;
          if (scanCount > 5)
          {
            // esp_deep_sleep_start();
            // return;
          }
          delay(3100);
          return;
        }
      }
    }
    else
    {
      if (!connected)
      {
        connected = connectToServer();
        if (!connected)
        {
          // Serial.println("Failed to connect...");
          // delay(3100);
          return;
        }
      }
    }

    if (cadence > 0)
    {
      unsigned long now = millis();
      runtime += now - last_millis;
      last_millis = now;
    }
    else
    {
      last_millis = millis();
    }

    // if the cadence is greater than 0, set the throttle HIGH and the LED LOW (inverted logic for the LED)
    // Serial.print("analog value: ");
    // Serial.println(analogRead(inductiveProbe));

    if (detectBraking(brakingTimeout))
    {
      pinMode(brakeOutput, OUTPUT);
      digitalWrite(brakeOutput, LOW);
      digitalWrite(PASoutput, LOW);
      digitalWrite(LED, HIGH);

      Serial.println("Braking");
    }
    else
    {
      pinMode(brakeOutput, INPUT);
      Serial.println("Not Braking");

      if (cadence > 0)
      {
        // digitalWrite(throttle, HIGH);
        digitalWrite(LED, LOW);
        bitBangPAS(cadence);
      }
      // if the cadence is 0, set the throttle LOW and the LED HIGH (inverted logic for the LED)
      else
      {
        // digitalWrite(throttle, LOW);
        digitalWrite(PASoutput, LOW);
        digitalWrite(LED, HIGH);
      }
    }
    // Serial.println(analogRead(brakeInput));

    // print the cadence to serial if it has changed using a new variable for the prev Value
    static int prevCadence = 0;
    if (cadence != prevCadence)
    {
      Serial.print("Cadence: ");
      Serial.println(cadence);
      prevCadence = cadence;
    }

    // loopUI();

    // delay(200); // Delay 200ms between loops.
  }
