

***

# E-Bike Pusher-Trailer Control System

## 📌 Project Overview
This project is a dual-core ESP32 firmware designed to control a powered bicycle trailer (pusher-trailer). The ESP32 communicates with a Bluetooth (BLE) cadence sensor to detect pedaling, reads an inductive brake probe on the trailer hitch, calculates a smoothed torque target, and sends high-speed CAN bus commands to a MKS ODrive Mini V1.0 motor controller.

The system features a Hybrid WiFi Web Dashboard (Home Network + Fallback AP Mode), Over-The-Air (OTA) updates, and custom non-blocking libraries for BLE and CAN communication.

---

## 🛠 Hardware Setup & Wiring

### Microcontroller: ESP32 (NodeMCU-32S / WROOM32D)
*   **Pin 17 (TX):** CAN Transceiver TX
*   **Pin 16 (RX):** CAN Transceiver RX
*   **Pin 25 (Input Pullup):** Inductive Brake Probe (Digital Signal: `HIGH` = Coasting, `LOW` = Trailer is pushing against the bike). *(Note: Moved from Pin 34 to 25 to utilize internal hardware pull-ups).*

### Motor Controller: MKS ODrive Mini V1.0 (ODrive v3.6 clone)
*   **Firmware:** ODrive v0.5.1
*   **Encoder:** AMS Absolute SPI (AS5047P) / CPR 16384 / Mode 257.
*   **CAN Bus:** Connected to ESP32 via 3.3V CAN Transceiver. **120-Ohm termination dip switch is ON.** Baudrate: `250000`. Node ID: `0`.

### Peripherals
*   **Cadence Sensor:** Standard BLE Cadence Sensor.
*   **Inductive Probe:** Digital switch mounted on the trailer hitch mechanism.

---

## 📂 Software Architecture & Libraries

The project relies on PlatformIO and uses pure ESP-IDF / Arduino core libraries to save memory and ensure high performance. **No external CAN libraries are used**; it utilizes the ESP32's native `twai.h` driver.

### 1. `src/main.cpp` (The Master Controller)
*   **Hybrid WiFi:** Attempts to connect to a saved Home network (default: `wlesswg`). If it fails after ~7 seconds, it spins up an Access Point (`ESP-Cadence`) and hosts a Captive Portal via `DNSServer`.
*   **OTA Updates:** Features a "Do Not Disturb" callback (`isUpdating`). Upon receiving an OTA request, it commands `0.0` torque and freezes the main loop to prevent `Errno 32: Broken pipe` socket crashes.
*   **Web Server:** REST API architecture. Endpoints (`/api/data`, `/api/log`, etc.) are mapped to named standard functions (e.g., `handleData()`) rather than inline lambdas to ensure stable compilation and memory management.

### 2. `include/dashboard.h` (The Web UI)
*   A pure HTML/CSS/JS frontend stored in flash (`PROGMEM`). 
*   **Features:** Real-time telemetry (Power, Cadence, Brake Factor), System Log console, EEPROM WiFi credential updates, and BLE Sensor scanning triggers.
*   **Network Stability:** The JS `fetch()` uses recursive timeouts (e.g., polling every 500ms *after* the previous request resolves) to prevent `errno: 11 (No more processes)` socket exhaustion on the ESP32.

### 3. `include/ODriveCAN.h` & `src/ODriveCAN.cpp`
*   A custom C++ wrapper for the ESP32 Native `twai.h` driver.
*   Configured for **Torque Control Mode** (`ControlMode = 1`, `InputMode = 1` Passthrough).
*   Requests encoder estimates and Phase Current (`IQC`) at 50Hz.

### 4. `include/CadenceSensor.h` & `src/CadenceSensor.cpp`
*   A custom BLE Client wrapper using the built-in `BLEDevice.h`.
*   **Key Fixes Applied:**
    *   *Stuck Cadence Bug:* Includes a 2000ms watchdog timer. If the sensor goes to sleep, cadence is forced to `0`.
    *   *16-bit Rollover:* Handles the 65536 crank-time rollover math natively.
    *   *Auto-Reconnect:* Gracefully handles `status=255` disconnects by automatically rebooting the client in the background without halting the main loop.

---

## 🧠 Core Control Logic (The Physics)

The system avoids "Bang-Bang" oscillation (violent jerking when the trailer pushes the bike) by separating the Cadence calculation from the Braking calculation.

1.  **Base Torque:** `Target Torque = Cadence (RPM) * CADENCE_TO_TORQUE_MULTIPLIER`.
2.  **Proportional Braking (EMA Filter):** The inductive probe is read at 50Hz and fed into an Exponential Moving Average (`alpha = 0.15`). 
    *   If the trailer lightly taps the bike, the `brake_avg` drops to `0.90`.
    *   The Target Torque is multiplied by this average, smoothly reducing motor power by 10% to let the bike pull away, avoiding a hard electrical shutoff.
3.  **Safety Timeout:** If the trailer pushes against the bike continuously for longer than `deviceInfo.brakingTimeout` (e.g., 2000ms), the `brake_avg` is instantly forced to `0.0` to trigger regenerative braking.

---

## ⚠️ Important Notes for the Next Agent/Developer

### 1. PlatformIO Partitions
Because BLE and WiFi consume massive amounts of Flash memory, the default partition table will fail to compile. The `platformio.ini` MUST include:
`board_build.partitions = min_spiffs.csv`
*(Note: If the partition table is ever modified, the subsequent flash MUST be done via USB, not OTA).*

### 2. Transitioning from Bench Test to E-Bike
The system is currently configured for a bench test using a **Gimbal Motor**. Before bolting this to the e-bike, the following changes MUST be made:
*   **ODrive Config:** Change `axis0.motor.config.motor_type` from `2` (Gimbal) to `0` (High Current). This will re-activate the ODrive's hardware shunt resistors so Phase Current (Amps) and Power (Watts) can be read correctly over CAN.
*   **Torque Multiplier:** In `main.cpp` `loop()`, change `CADENCE_TO_TORQUE_MULTIPLIER = 0.01` to a higher value appropriate for the hub motor (e.g., `0.1` or `0.2`).
*   **Torque Constant:** In `main.cpp` `handleData()`, update the physics math: `(odrive.getCurrent() * 0.356)`. Replace `0.356` with the hub motor's actual torque constant (`8.27 / KV`).

### 3. ODrive Startup Calibration
The AS5047P encoder offset has been permanently saved to the ODrive's memory (`pre_calibrated = True`), bypassing the 90-degree boot sweep. **If the magnet on the back of the motor is ever physically rotated or removed**, the offset calibration *must* be re-run in `odrivetool` (`odrv0.axis0.requested_state = 7`), otherwise the motor will violently stutter.

### 4. Code Generation Quirks (AI Formatting)
In earlier iterations, defining inline lambdas for the Web Server `server.on()` endpoints caused the LLM parser to falsely trigger an "End of Sequence" token due to the `