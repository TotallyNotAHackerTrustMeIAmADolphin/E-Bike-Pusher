***

# 🚲 E-Bike Autonomous Pusher-Trailer

## 📌 Project Overview
This project controls an autonomous E-Bike "Pusher" Trailer using an **ESP32**, an **MKS ODrive Mini V1.0**, and a 1000W Direct-Drive Hub Motor. 

Instead of traditional throttles, the trailer uses a **Digital Inductive Hitch Probe** and a **Bluetooth (BLE) Cadence Sensor**. The ESP32 mathematically converts the digital hitch switch into a smooth analog signal, feeding it into a custom **PID Velocity Controller**. The result is a trailer that completely "disappears," seamlessly speed-matching the bicycle, coasting down hills, and aggressively regen-braking when the bike slows down.

The system features a **Web Bluetooth (WebBLE) Dashboard** for live PID tuning on the road, and an Over-The-Air (OTA) maintenance mode for wireless firmware updates.

---

## 🛠 Hardware & Wiring

### 1. Microcontroller (ESP32 NodeMCU/WROOM32D)
*   **GPIO 17 (TX) & GPIO 16 (RX):** 3.3V CAN Bus Transceiver (e.g., SN65HVD230).
*   **GPIO 34 (Input):** Inductive Brake Probe Signal. 
*   **GPIO 33 (Output HIGH):** Dedicated 3.3V Power Bus. 
    *   *Hardware Hack:* GPIO 34 lacks internal pull-up resistors. A physical 10kΩ resistor connects GPIO 33 to GPIO 34 to prevent the pin from floating when the 12V inductive sensor diode is reverse-biased.

### 2. Motor Controller (MKS ODrive Mini V1.0)
*   **Firmware:** Factory v0.5.1. *(⚠️ **WARNING:** Do NOT flash official ODrive firmware onto this board. MKS pin-routing differences will permanently brick the CAN transceiver!)*
*   **CAN Bus:** 250k baud, Node ID: 0. **120Ω Termination DIP switch MUST be ON.**
*   **Motor Setup:** 1000W Direct Drive Hub Motor. **23 Pole Pairs** (46 magnets), Hall Effect Sensors.
*   **Power:** 24V Battery stepped up via Boost Converter.

---

## 🧠 Control Theory & Physics

The ESP32 completely bypasses the ODrive's internal trajectory planners and runs its own 50Hz physics engine.

### 1. The "Digital-to-Analog" Hitch Filter
The inductive probe on the trailer hitch is a pure digital switch: it is either compressed (pushing the bike) or extended (lagging behind). Using raw digital inputs causes violent "Bang-Bang" oscillation. 

To fix this, the ESP32 reads the probe at 50Hz and passes it through an **Exponential Moving Average (EMA) Low-Pass Filter**.
*   The raw state is mapped to **-1.0 (Compressed/Braking)** and **1.0 (Extended/Pushing)**.
*   The filter uses a `brakeTimeConstant` (e.g., 1.0 seconds) defined by the user in the Dashboard.
*   *Result:* We get `brake_avg`, a buttery-smooth floating-point variable that smoothly glides between `-1.0` and `1.0`.

### 2. The 3-Zone State Machine
The ESP32 dynamically switches the ODrive's internal physics mode on the fly over the CAN bus to mimic how a car works:

*   🔴 **ZONE 1: Active Braking** (`brake_avg < -0.5`)
    *   **Mode:** Torque Control (`1`).
    *   **Action:** Proportional Regenerative Braking. As the hitch compresses further, negative Amps are sent to the motor.
    *   **Safety Lock:** Braking is *only* permitted if the wheel velocity is `> 0.05 rev/s`. This prevents the trailer from rolling backward at a stoplight.
*   ⚪ **ZONE 2: Coasting / Deadband** (`brake_avg` between `-0.5` and `0.1` OR `cadence == 0`)
    *   **Mode:** Torque Control (`1`).
    *   **Action:** Commands `0.0A`. The motor acts as a perfect freewheel. 
*   🟢 **ZONE 3: Speed-Matching Push** (`brake_avg > 0.1` AND `cadence > 0`)
    *   **Mode:** Velocity Control (`2`).
    *   **Action:** The Custom PID Controller engages.

### 3. The Custom PID Velocity Controller
When in Zone 3, the ESP32 calculates a `target_velocity` to send to the ODrive. The `brake_avg` acts as the **Error** for the PID loop.

*   **P (Proportional - The "Punch"):** Reacts instantly to how far the hitch is stretched. Helps the trailer launch aggressively from a stoplight.
*   **I (Integral - The "Cruising Speed"):** Slowly winds up over time to match the bicycle's exact cruising speed. 
    *   *Anti-Windup:* Whenever the trailer enters Zone 1 (Braking) or Zone 2 (Coasting), the Integrator (`I_out`) is explicitly anchored to the wheel's `actual_velocity`. This ensures that when the user resumes pedaling, the motor picks up *exactly* where it left off with zero jerk or stall.
*   **D (Derivative - The "Shock Absorber"):** Reacts to how fast the hitch is moving. Dampens the target velocity to prevent the trailer from oscillating (spring-mass hunting) against the bike.
*   **Speed Clamp:** Hard-capped at `max_speed` (e.g., 3.2 rev/s ≈ 25 km/h for a 28" wheel).

---

## 💻 Software Architecture

*   **PlatformIO / Arduino Core v3.0+:** Built using `min_spiffs.csv` partitions to fit the massive Bluetooth libraries in flash memory.
*   **NimBLE-Arduino:** The default ESP32 `Bluedroid` stack causes RAM fragmentation and crashes when acting as a Server (Dashboard) and Client (Cadence) simultaneously. The lightweight `NimBLE` library completely cures this.
*   **WebBLE Dashboard:** The UI (`index.html`) is hosted externally on GitHub Pages. It connects directly from a smartphone to the ESP32 via Bluetooth. Live tuning commands (`CFG:`) are applied instantly to RAM. Clicking "Save" commits them to EEPROM.
*   **Cadence Watchdog:** The Cadence sensor scans in a non-blocking background thread. If it drops, the ESP32 waits 2 seconds before forcing Cadence to 0. 

---

## ⚙️ ODrive Configuration Quirks

Due to the heavy hub motor and low-resolution Hall sensors, the ODrive requires specific tuning to prevent 30A current spikes and `DC_BUS_OVER_CURRENT` crashes.

Run these in `odrivetool` over USB during initial setup:

```python
# 1. Hall Sensor Config (Allow sloppy factory tolerances)
odrv0.axis0.encoder.config.mode = 1
odrv0.axis0.motor.config.pole_pairs = 23
odrv0.axis0.encoder.config.cpr = 138
odrv0.axis0.encoder.config.calib_range = 0.05
odrv0.axis0.encoder.config.bandwidth = 50.0

# 2. Calm the Current Controller (Prevents 30A Spikes!)
odrv0.axis0.motor.config.current_control_bandwidth = 20.0
odrv0.axis0.motor.config.requested_current_range = 60.0
odrv0.axis0.motor.config.current_lim = 25.0

# 3. Soften Velocity PI (ESP32 handles the macro-PID)
odrv0.axis0.controller.config.vel_gain = 0.05
odrv0.axis0.controller.config.vel_integrator_gain = 0.01

# 4. Enable Watchdog (Deadman Switch)
# If ESP32 crashes or CAN breaks, motor disarms in 0.5s.
odrv0.axis0.config.enable_watchdog = True
odrv0.axis0.config.watchdog_timeout = 0.5

# Save permanently
odrv0.save_configuration()
```

*(Note: The MKS v0.5.1 firmware lacks State 12 Hall Polarity calibration. If `CPR_POLEPAIRS_MISMATCH` occurs during setup, you must manually force `odrv0.axis0.motor.config.direction = 1` or `-1` before running State 7).*

---

## 🚀 OTA Updates (Maintenance Mode)
To prevent `Errno 104: Connection reset by peer` during OTA flashes, WiFi is completely disabled during normal riding.
1.  Connect to the WebBLE Dashboard.
2.  Click **OTA Maintenance Mode**.
3.  The ESP32 saves a flag to EEPROM, shuts down the motor, kills the Bluetooth radio, and reboots.
4.  Upon reboot, it connects to the Home WiFi network (or creates an AP) and idles securely, waiting for a PlatformIO flash.