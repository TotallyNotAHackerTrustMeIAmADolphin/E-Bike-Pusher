

***

# 🚲 E-Bike Autonomous Pusher-Trailer

## 📌 Project Overview
This project is a custom ESP32-based control system for an autonomous E-Bike "Pusher" Trailer. Instead of using a traditional handlebar throttle, the system acts as a robotic follower. It calculates the physical equilibrium between the bicycle and the trailer using a **Digital Inductive Hitch Sensor**, pairs it with a **Bluetooth (BLE) Cadence Sensor** to ensure legal pedaling, and commands a 1000W Hub motor to seamlessly push the rider.

The core firmware relies heavily on custom PID Control Theory to dynamically switch between **Velocity Control** (for buttery-smooth pushing) and **Torque Control** (for freewheeling and active regenerative braking) at 50Hz over a CAN bus.

---

## 🛠 Hardware Setup & Wiring

### 1. Microcontroller: ESP32 (NodeMCU-32S / WROOM32D)
*   **GPIO 17 (TX) & GPIO 16 (RX):** Connected to a 3.3V CAN Transceiver (e.g., SN65HVD230).
*   **GPIO 34 (Input):** Inductive Brake Probe Signal. 
    *   *Logic:* Technically a digital open-collector sensor that pulls to GND. However, it is read via `analogRead()` because the voltage swing can be marginal near the ESP32's logic thresholds. Analog sampling allows for custom hysteresis and sensor health monitoring.
*   **GPIO 33 (Output HIGH):** Acts as a dedicated 3.3V power bus. Connected to GPIO 34 via a physical **10kΩ Pull-Up Resistor**.

### 2. Motor Controller: MKS ODrive Mini V1.0
*   **Hardware:** Clone of ODrive v3.6 (56V architecture).
*   **Firmware:** Factory v0.5.1. *(⚠️ **WARNING:** Do NOT flash official ODrive firmware onto this board. MKS pin-routing differences will permanently brick the CAN transceiver!)*
*   **CAN Bus:** Connected to the ESP32. The tiny **120R Termination DIP switch MUST be ON**. Baudrate: `250000`. Node ID: `0`.
*   **Motor:** 1000W Direct Drive Hub Motor (28-inch wheel, 45mm tire). **23 Pole Pairs** (46 magnets) with Hall Effect Sensors.

### 3. Power Electronics
*   **Battery:** 24V (or 36V/48V) Lithium Battery.
*   **Boost Converter:** Steps up voltage if required. *(⚠️ **CRITICAL:** Because boost converters block reverse current, 100% of regenerative braking energy is dumped into the ODrive's Brake Resistor. Monitor resistor temps closely!)*

---

## 📱 The WebBLE Dashboard & Live Tuning

To eliminate the "Radio Coexistence" bug (where WiFi and Bluetooth fight for the ESP32's antenna and crash the board), **WiFi is permanently disabled during riding.**

The interface is hosted entirely via **Web Bluetooth (WebBLE)**.
👉 **[Access the Dashboard Here](https://totallynotahackertrustmeiamadolphin.github.io/E-Bike-Pusher/)**

### How to use it:
1. Open the link in a WebBLE-compatible browser (Chrome/Edge/Brave on PC/Android, or **Bluefy** on iOS).
2. Click **Connect to Bike**. The dashboard will instantly populate with your saved tuning parameters and begin streaming 2Hz telemetry.
3. **Live Tuning:** Adjust the PID or Threshold values. The ESP32 instantly applies them to RAM for live test-riding.
4. **Save to Flash:** Locks your perfect tune into the ESP32's EEPROM permanently.

---

## 🧠 Control Theory & Physics (The "Equilibrium" Controller)

The magic of this system is how it converts a harsh binary sensor into a buttery-smooth proportional speed-matcher.

### 1. The Digital-to-Analog Filter (`brake_avg`)
The inductive probe is a digital switch: it is either `1` (Hitch Extended) or `-1` (Hitch Compressed). 
At 50Hz, the ESP32 feeds this `1` or `-1` state into an **Exponential Moving Average (EMA) Low-Pass Filter**. The filter's speed is dictated by the `Filter Time (s)` parameter on the dashboard.
*   The result is `brake_avg`: A floating-point number that slides smoothly between `-1.0` and `1.0`.
*   If the trailer lightly taps the bike, `brake_avg` drops to `0.8`. If it crashes into the bike, it plunges to `-1.0`.

### 2. The 3-Zone State Machine
The ESP32 reads `brake_avg` and dynamically changes the ODrive's physics mode on the fly:

*   **ZONE 1: Active Braking (`brake_avg < -0.5`)**
    *   *ODrive Mode:* Torque Control.
    *   *Action:* Applies proportional negative torque (Regen Braking). Maxes out at -15.0A.
    *   *Safety:* Regenerative braking is strictly disabled if the wheel's forward velocity drops below `0.05 rev/s`. This prevents the trailer from pulling the bicycle backward at a stoplight.
    *   *Anti-Jerk:* The internal velocity integrator (`I_out`) is continuously anchored to the wheel's actual rolling speed so the system is ready to resume pushing seamlessly.

*   **ZONE 2: Coasting & Deadband (`cadence == 0` AND `brake_avg > -0.5`)**
    *   *ODrive Mode:* Torque Control.
    *   *Action:* Commands `0.0A` of torque, allowing the motor to perfectly freewheel. The velocity integrator remains anchored.

*   **ZONE 3: PID Velocity Push (`cadence > 0` AND `brake_avg > -0.5`)**
    *   *ODrive Mode:* Velocity Control.
    *   *Action:* The trailer becomes a Speed-Matcher. It uses a **PID Controller** where `Error = brake_avg`.
    *   **P-Term (Kp):** The instant "Punch." Applies immediate speed when the hitch stretches.
    *   **I-Term (Ki):** The "Acceleration." Slowly builds up the target speed to match the bicycle's cruising velocity.
    *   **D-Term (Kd):** The "Shock Absorber." Reacts to rapid changes in the hitch to prevent back-and-forth oscillation (surging).

---

## ⚙️ Initial ODrive Configuration (via `odrivetool`)

A virgin MKS ODrive Mini must be configured for a low-inductance, heavy Hub Motor. The native PID loops must be softened to prevent `DC_BUS_OVER_CURRENT` spikes.

```python
# 1. Hall Sensor & Motor Math (Example: 23 Pole Pairs)
odrv0.axis0.encoder.config.mode = 1
odrv0.axis0.motor.config.pole_pairs = 23
odrv0.axis0.encoder.config.cpr = 138

# 2. Relax Hall Tolerances (Crucial for sloppy hub motors)
odrv0.axis0.encoder.config.calib_range = 0.05
odrv0.axis0.encoder.config.bandwidth = 50.0

# 3. Soften Current Controller (Prevents 30A ringing spikes)
odrv0.axis0.motor.config.current_control_bandwidth = 20.0

# 4. Soften Internal Velocity PI (Prevents jitter in Velocity Mode)
odrv0.axis0.controller.config.vel_gain = 0.05
odrv0.axis0.controller.config.vel_integrator_gain = 0.01

# 5. Boot Settings & Watchdog
odrv0.axis0.config.startup_encoder_offset_calibration = False
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.axis0.config.enable_watchdog = True
odrv0.axis0.config.watchdog_timeout = 0.5
```
*Note: Makerbase (MKS) factory firmware v0.5.1 lacks the Hall Polarity Calibration step (State 12). If you get `CPR_POLEPAIRS_MISMATCH` errors during initial setup, motor direction must be forced manually (`odrv0.axis0.motor.config.direction = 1` or `-1`) before running State 7.*

---

## 📡 OTA Updates & Maintenance Mode

To flash new firmware Over-The-Air, click **"OTA Mode"** on the WebBLE Dashboard.
1. The ESP32 saves a flag to EEPROM, safely disarms the ODrive motor, and reboots.
2. Upon waking, the ESP32 bypasses all BLE and Motor logic.
3. It spins up its WiFi radio, connects to your saved Home SSID (or creates a fallback AP named `ESP-Maintenance`), and waits for a PlatformIO OTA payload. 

## 🔋 Safety Auto-Revive
To conserve battery, the BLE Cadence Sensor goes to sleep when pedaling stops. The ESP32 utilizes the `NimBLE` library to run non-blocking background scans to re-pair with the sensor. 
If scanning causes the ESP32 to drop CAN bus packets for >0.5 seconds, the ODrive's hardware Watchdog will kill the motor. The ESP32 actively monitors the ODrive's CAN Heartbeat; if it detects the motor has disarmed, it automatically issues a `CLEAR_ERRORS` command and re-arms the drive system to ensure seamless riding.
