# 🚲 ESP-Cadence: E-Bike Autonomous Pusher-Trailer

This project is a custom ESP32 firmware for an autonomous E-Bike "Pusher" Trailer. It uses a **Digital Inductive Hitch Sensor** and a **BLE Cadence Sensor** to control a 1000W Hub motor via an **MKS ODrive Mini V1** (ODrive v3.6 clone) running **v0.5.1 firmware** over **CAN bus**.

## 📌 Project Overview

*   **Platform:** ESP32 (NodeMCU-32S)
*   **Framework:** Arduino / PlatformIO
*   **Core Logic:** 50Hz PID Velocity Controller that matches the trailer's speed to the bicycle by maintaining hitch equilibrium.
*   **Hardware:** MKS ODrive Mini V1 (Use ODriveTool 0.5.x). *Warning: Official ODrive firmware may brick the MKS CAN transceiver.*
*   **Communication:** 
    *   **CAN Bus (TWAI):** 250kbps to ODrive motor controller.
    *   **BLE:** NimBLE-based interface for Cadence Sensors and a WebBLE Dashboard.
    *   **WiFi:** Only active in "Maintenance Mode" for OTA updates.

## 🛠 Building and Running

### Prerequisites
*   [PlatformIO IDE](https://platformio.org/)
*   ESP32 hardware (NodeMCU-32S or equivalent)
*   SN65HVD230 CAN Transceiver (or similar)

### Key Commands
*   **Build:** `pio run`
*   **Upload (USB):** `pio run -t upload`
*   **Serial Monitor:** `pio device monitor -b 115200`
*   **Upload (OTA):** 
    1. Activate "OTA Mode" in the Dashboard.
    2. Run: `pio run -t upload --upload-port <ESP_IP_ADDRESS>` (Update IP in `platformio.ini`).

## 🧠 Software Architecture

### Key Components
*   **`src/main.cpp`**: The heart of the system. Implements the 50Hz control loop, 3-zone state machine (Braking, Coasting, Pushing), and robust error recovery logic.
    *   **Inductive Sensor Logic:** The sensor is a digital open-collector type. It is read via `analogRead()` on GPIO 34 to handle marginal logic levels and provide software-based hysteresis for a smoother transition between states.
*   **`ODriveCAN` (`include/ODriveCAN.h`)**: A lightweight wrapper for the ESP32 TWAI driver to communicate with ODrive v3.6/MKS ODrive Mini.
*   **`CadenceSensor` (`include/CadenceSensor.h`)**: Manages the BLE connection to standard Cycling Speed and Cadence (CSC) sensors. Handles scanning, pairing, and RPM calculation.
*   **`BLEDashboard` (`include/BLEDashboard.h`)**: Provides a Web Bluetooth interface for live PID tuning, telemetry streaming (2Hz), and system commands (OTA, Scan, Save).

### Control Logic (The 3 Zones)
1.  **Zone 1: Active Braking**: If `brake_avg < -0.5`, applies proportional regenerative braking torque (up to -15A).
2.  **Zone 2: Coasting**: If not pedaling or hitch is slightly compressed, commands 0.0A torque for freewheeling.
3.  **Zone 3: PID Pushing**: If pedaling and hitch is extended, uses a PID controller where `Error = brake_avg`. Target is `brake_avg = 0.0`.

## ⚙️ Configuration & Storage

*   **EEPROM**: Settings are stored in a `DeviceInfo` struct starting at `EEPROM_ADDRESS = 0`. This includes PID gains (`Kp`, `Ki`, `Kd`), speed limits, and BLE/WiFi credentials.
*   **Initial Setup**: The system resets to defaults if EEPROM is uninitialized.
*   **ODrive Config**: A virgin ODrive must be configured via `odrivetool` (see `README.md` for specific commands).

## ⚠️ Safety & Conventions

*   **No WiFi while Riding**: WiFi is strictly disabled during normal operation to prevent interference with BLE and CAN (Radio Coexistence bug).
*   **Watchdog Recovery**: The ESP32 monitors ODrive heartbeats and will automatically attempt to re-arm the motor if it disarms due to a watchdog timeout. Critical hardware errors (over-current, etc.) prevent auto-revive.
*   **Regen Safety**: Regenerative braking is disabled below 0.05 rev/s to prevent the trailer from moving backward.
