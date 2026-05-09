***

# 🚲 E-Bike Autonomous Pusher-Trailer

## 📌 Project Overview
This project is a custom ESP32-based control system for an autonomous E-Bike "Pusher" Trailer. Rather than using a traditional throttle or PAS sensor wired directly to a controller, this system uses an **Inductive Hitch Sensor** and a **Bluetooth (BLE) Cadence Sensor** to calculate the physical equilibrium between the bicycle and the trailer.

It commands a 1000W Direct-Drive Hub motor using an **MKS ODrive Mini V1.0** over a high-speed CAN bus, dynamically switching between **Velocity Control** (for smooth pushing) and **Torque Control** (for freewheeling and active regenerative braking) at 50Hz.

## 🛠 Hardware Setup & Wiring

### 1. Microcontroller (ESP32 NodeMCU/WROOM32D)
*   **GPIO 17 (TX):** CAN Transceiver TX
*   **GPIO 16 (RX):** CAN Transceiver RX
*   **GPIO 34 (Input):** Inductive Brake Probe Signal. *(Reads `LOW` when the trailer pushes against the bike hitch).*
*   **GPIO 33 (Output HIGH):** Acts as a dedicated 3.3V power bus. Connected to GPIO 34 via a physical **10kΩ Pull-Up Resistor** (Required because GPIO 34 lacks internal pull-up silicon).

### 2. Motor Controller (MKS ODrive Mini V1.0)
*   **Hardware:** Clone of ODrive v3.6 (56V architecture).
*   **Firmware:** Factory v0.5.1. *(⚠️ **WARNING:** Do NOT flash official ODrive firmware onto this board. MKS pin-routing differences will permanently brick the CAN transceiver!)*
*   **CAN Bus:** Connected to ESP32 via a 3.3V CAN Transceiver (e.g., SN65HVD230). The tiny **120R Termination DIP switch MUST be ON**.
*   **Motor:** 1000W Direct Drive Hub Motor (28-inch wheel, 45mm tire). **23 Pole Pairs** (46 magnets).

### 3. Peripherals
*   **BLE Cadence Sensor:** Standard Bluetooth crank sensor.
*   **Power:** 24V Battery stepped up via a Boost Converter. (Regen energy *must* be burned by the ODrive Brake Resistor to protect the Boost Converter).

---

## 🧠 Control Theory & Physics (The "Equilibrium" Controller)

To prevent the trailer from aggressively jerking the bike ("Bang-Bang" oscillation), the system uses an **Integral Velocity Speed-Matcher** governed by a Proportional Low-Pass Filter.

### 1. The Hitch Filter (`brake_avg`)
The digital inductive probe on the hitch is polled at 50Hz. Its state (-1.0 for compressed, 1.0 for extended) is fed through an Exponential Moving Average (EMA). The smoothing time is controlled by the Dashboard's **Filter Time (s)** setting. 
*   `1.0` = Trailer is falling behind (Maximum Push).
*   `0.0` = Perfect equilibrium (Coasting).
*   `-1.0` = Trailer is crashing into the bike (Maximum Brake).

### 2. The 3-Zone State Machine
The ESP32 dynamically changes the ODrive's internal physics mode on the fly:

*   **ZONE 1: Pushing (Hitch Extended + Pedaling)**
    *   *ODrive Mode:* Velocity Control.
    *   *Action:* The trailer accelerates. The acceleration rate is proportional to the hitch extension (`brake_avg`) multiplied by the Dashboard's **Accel Rate**. Max speed is hardcoded to 25 km/h (3.2 rev/s).
*   **ZONE 2: Coasting (Hitch Neutral OR Not Pedaling)**
    *   *ODrive Mode:* Torque Control.
    *   *Action:* Commands `0.0A` of torque, allowing the motor to perfectly freewheel. Behind the scenes, the ESP32 synchronizes its internal `target_velocity` to the wheel's actual rolling speed so the next push starts seamlessly.
*   **ZONE 3: Active Braking (Hitch Compressed)**
    *   *ODrive Mode:* Torque Control.
    *   *Action:* Applies proportional negative torque (Regen Braking). 
    *   *Safety:* Regenerative braking is strictly disabled if the wheel's forward velocity drops below `0.05 rev/s`. This prevents the trailer from pulling the bicycle backward at a red light.

---

## 💻 Software Architecture

The codebase is built on **PlatformIO** using the ESP-IDF/Arduino Core v3.0+. 

*   **`main.cpp`:** The master 50Hz control loop. Handles the control physics, ODrive CAN integration, and the OTA Maintenance Mode transition.
*   **`BLEDashboard.h/cpp`:** Uses the **NimBLE** library to host a Web Bluetooth GATT Server. Exposes 3 Characteristics (RX, TX, LOG) to stream telemetry at 2Hz and accept live-tuning commands.
*   **`CadenceSensor.h/cpp`:** A NimBLE Client. Safely scans the room in a non-blocking background thread, handles the 16-bit crank time rollover math, and employs a 2-second watchdog timeout if the sensor goes to sleep.
*   **`ODriveCAN.h/cpp`:** A lightweight, custom C++ wrapper for the ESP32's native `twai.h` CAN driver. (No external libraries required).
*   **`index.html`:** The user interface. Hosted externally on GitHub Pages. Uses the Web Bluetooth API to connect directly from a smartphone browser to the ESP32.

---

## ⚙️ Compilation & Setup

### PlatformIO Configuration (`platformio.ini`)
Because Bluetooth libraries consume massive amounts of Flash memory, the default ESP32 partition table will fail to compile. The project requires `min_spiffs.csv`.

```ini
[env:nodemcu-32s]
platform = espressif32
board = nodemcu-32s
framework = arduino
monitor_speed = 115200
board_build.partitions = min_spiffs.csv
lib_deps = h2zero/NimBLE-Arduino@^1.4.3
```

### ODrive Hub-Motor Configuration
The MKS ODrive Mini requires specific terminal configurations to drive a heavy Hub Motor over Hall Sensors without violent current spikes (`DC_BUS_OVER_CURRENT`).

```python
# Force Hall Sensor Mode & 23 Pole Pairs
odrv0.axis0.encoder.config.mode = 1
odrv0.axis0.motor.config.pole_pairs = 23
odrv0.axis0.encoder.config.cpr = 138

# Relax Hall Tolerances (Crucial for cheap hub motors)
odrv0.axis0.encoder.config.calib_range = 0.05
odrv0.axis0.encoder.config.bandwidth = 50.0

# Prevent 30A PI-Loop Oscillations
odrv0.axis0.motor.config.current_control_bandwidth = 20.0

# Watchdog Deadman Switch (0.5 seconds)
odrv0.axis0.config.enable_watchdog = True
odrv0.axis0.config.watchdog_timeout = 0.5
```
*Note: Makerbase (MKS) factory firmware removed the Hall Polarity Calibration step (State 12). To fix `CPR_POLEPAIRS_MISMATCH` errors during initial setup, motor direction must be forced manually (`odrv0.axis0.motor.config.direction = 1` or `-1`).*

---

## 📱 The WebBLE Dashboard & OTA Updates

To eliminate the "Radio Coexistence" bug (where WiFi and BLE fight for the ESP32's single antenna and crash the board), **WiFi is permanently disabled during riding.**

1.  **Connecting:** Open `index.html` on a WebBLE-compatible browser (Chrome on Android/PC, Bluefy on iOS). Click "Connect to Bike". 
2.  **Live Tuning:** Adjusting the "Accel Rate" or "Filter Time" inputs will instantly apply to the bike's RAM for live test-riding. Clicking "Save to Flash" commits the tuning to the ESP32's EEPROM.
3.  **OTA Maintenance Mode:** Clicking "OTA Mode" commands the ESP32 to completely shut down the motor and the BLE radio. The ESP32 will reboot, turn on its WiFi radio, connect to the saved Home Router (or spin up an AP named `ESP-Maintenance`), and await a firmware flash from PlatformIO.

---

## 🐛 Known Quirks & Fixes
*   **BLE Buffer Crashing (`rc=-1`):** Caused by typing into the dashboard too fast. Fixed by a Javascript 500ms debounce timer on the input fields.
*   **Dual-Role BLE Freeze (`Failed to allocate 0 bytes`):** The standard ESP32 Bluedroid stack cannot handle being a Server and a Client simultaneously. **NimBLE** must be used.
*   **Cadence Sensor Blocking:** If the sensor goes to sleep, blind connection attempts will hang the main loop. The `CadenceSensor` class enforces a 2-second background scan using `_scanner->start(2, scanEndedCB, false)` to "peek" into the room before attempting a connection.
*   **Motor Spins at ESP32 Reset:** Because the ODrive has an independent power supply, restarting the ESP32 leaves the ODrive executing its last received command. Fixed by enabling the ODrive's hardware CAN Watchdog.