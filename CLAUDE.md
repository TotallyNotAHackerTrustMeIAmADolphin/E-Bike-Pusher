# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32 firmware (Arduino/PlatformIO) for an autonomous E-Bike "Pusher" Trailer. It reads a digital
inductive hitch sensor and a BLE Cycling Speed & Cadence sensor, runs a 50Hz control loop, and drives
a 1000W hub motor through an MKS ODrive Mini V1 (ODrive v3.6 clone, factory firmware v0.5.1) over CAN
(TWAI). Tuning and telemetry are exposed to a WebBLE dashboard (`index.html`,
hosted via GitHub Pages at the repo's Pages URL).

There is no application-level test suite (`test/` only contains PlatformIO's placeholder README) and
no linter is configured — validation is build success plus on-hardware behavior.

## Build & Run Commands

This is a PlatformIO project (`platformio.ini`, env `nodemcu-32s`, `board_build.partitions = min_spiffs.csv`).

- Build: `pio run`
- Upload over USB: `pio run -t upload`
- Serial monitor: `pio device monitor -b 115200`
- Upload OTA: put the device in "OTA Mode" via the dashboard first, then
  `pio run -t upload --upload-port <ESP_IP_ADDRESS>` (or update `upload_port` in `platformio.ini`,
  which is currently set to a static IP — the commented `odrive-node.local` mDNS name is the fallback).

There are no unit tests to run and no lint/format command in this repo.

## Architecture

### Source layout
- `src/main.cpp` — entry point: `setup()`/`loop()`, the 50Hz control loop, the 3-zone state machine,
  hitch sensor filtering, ODrive watchdog/auto-revive logic, and EEPROM load/save triggers.
- `include/ODriveCAN.h` / `src/ODriveCAN.cpp` — thin wrapper around the ESP32 TWAI (CAN) driver
  implementing the ODrive v0.5.x CAN protocol (node ID `0`, 250 kbps). Exposes mode/state/torque/velocity
  setters, telemetry getters, and heartbeat freshness (`isDataFresh()`) for watchdog logic.
- `include/CadenceSensor.h` / `src/CadenceSensor.cpp` — NimBLE central that scans for and connects to a
  standard BLE CSC (Cycling Speed and Cadence) sensor, computes RPM from crank revolution notifications,
  and can enter a one-shot pairing scan mode that reports a newly found device back to `main.cpp`.
- `include/BLEDashboard.h` / `src/BLEDashboard.cpp` — NimBLE peripheral ("E-Bike Pusher") exposing a
  GATT service with TX (telemetry, NOTIFY), RX (commands, WRITE), and LOG (NOTIFY) characteristics.
  Also owns the `DeviceInfo` struct that is the EEPROM-persisted config record.
- `index.html` — the standalone Web Bluetooth dashboard (no build step; static file served via GitHub
  Pages) that connects to the BLE service above for live tuning and telemetry.
- `ODrive_config.json` — reference ODrive configuration/state dump, not consumed by the firmware.

### Control loop (`src/main.cpp`, 50Hz / every 20ms)

1. **Hitch sensing (`updateBrakeLogic`)**: GPIO 34 is read with `analogRead()` (not `digitalRead()`)
   because the open-collector sensor's voltage swing is marginal near logic thresholds; this also
   allows software hysteresis (thresholds at 1800/2200 out of 4095) and a dead-man range check
   (<200 or >3900 reads as a sensor failure and forces a neutral/safe state). The debounced digital
   state (`1` extended / `-1` compressed / `0` fault) is fed through an EMA low-pass filter
   (`brakeTimeConstant`, tunable) to produce `brake_avg`, a float in roughly `[-1.0, 1.0]`.
2. **3-zone state machine**, evaluated every tick against `brake_avg` and cadence:
   - **Zone 1 — Active Braking** (`brake_avg < -0.5`): ODrive switches to Torque Control, applies
     proportional regen torque up to -15A, but only while `actual_velocity > 0.05 rev/s` (prevents
     pulling the bike backward at a stop). The velocity integrator (`I_out`) is kept anchored to actual
     wheel speed so Zone 3 can resume smoothly.
   - **Zone 2 — Coasting** (`cadence == 0` and `brake_avg > -0.5`): Torque Control, 0.0A (freewheel),
     integrator still anchored.
   - **Zone 3 — PID Velocity Push** (`cadence > 0` and `brake_avg > -0.5`): Velocity Control; a PID
     loop where `error = brake_avg` and target is `brake_avg == 0` (hitch equilibrium). On entry from
     another zone the integrator is re-seeded (`I_out = actual_velocity - Kp * brake_avg`) to avoid a
     surge. Output is clamped to `[0, max_speed]`.
   - If the ODrive CAN heartbeat isn't fresh (`isDataFresh()` false), torque is forced to 0 regardless
     of zone — this is the primary loss-of-communication safety path.
3. **ODrive watchdog/auto-revive**: the ESP32 polls ODrive state/error over CAN; if the axis is not in
   closed-loop state (8) it checks the error code every 2s and, if the only error is
   `ODRV_ERROR_WATCHDOG_TIMER_EXPIRED` (or none), issues `clearErrors()` + `setState(8)` to re-arm
   automatically. Any other error is logged as critical and left disarmed (no auto-revive).
4. Telemetry is pushed to the BLE dashboard every 500ms via `dash_sendTelemetry`.

### Configuration & persistence

- `DeviceInfo` (in `BLEDashboard.h`) is the single struct persisted to EEPROM at `EEPROM_ADDRESS = 0`
  (`EEPROM_SIZE = 256`): PID gains (`vel_Kp/Ki/Kd`), `max_speed`, `brakeTimeConstant`, saved BLE cadence
  sensor MAC/name/address type, WiFi STA credentials, and a `maintenanceMode` flag.
- On boot, if key fields are NaN/uninitialized, `main.cpp` resets `DeviceInfo` to hardcoded defaults and
  writes it back — this is the only "factory reset" path.
- The dashboard's RX characteristic accepts plain-text commands: `OTA`, `SCAN`, `SAVE`, `GET` (returns a
  JSON config blob), `CFG:kp:ki:kd:ms:tc` (live-tunes PID/speed/filter in RAM), and `WIFI:ssid:pass`.
  Any protocol change here must be mirrored in `index.html`.

### WiFi / OTA maintenance mode

WiFi is normally fully disabled to avoid the ESP32's WiFi/BLE radio-coexistence bug interfering with the
BLE cadence link and CAN timing. Triggering "OTA Mode" from the dashboard sets `maintenanceMode` in
EEPROM, disarms the ODrive, and reboots; `runMaintenanceMode()` then skips BLE/motor init entirely,
connects to the saved home SSID (falling back to a `ESP-Maintenance` softAP), and blocks in an
`ArduinoOTA.handle()` loop until a new firmware image is pushed. There is no return path to normal mode
without a re-flash/reboot.

## Hardware context worth knowing when touching related code

- ESP32 (NodeMCU-32S): CAN TX/RX on GPIO 17/16 through an external 3.3V transceiver; hitch probe on
  GPIO 34 (analog input) with GPIO 33 driven HIGH as a dedicated pull-up power rail.
- MKS ODrive Mini V1 running factory firmware v0.5.1 — **do not** assume upstream ODrive firmware
  behavior/CAN IDs; MKS's pin routing differs and flashing official ODrive firmware can brick the CAN
  transceiver. Motor is a 23-pole-pair hub motor; encoder mode/CPR and current-controller bandwidth were
  hand-tuned for this motor (see `README.md` for the `odrivetool` config used to provision a fresh unit).
- Boost converters between the battery and ODrive block reverse current, so 100% of regen energy goes
  into the ODrive's brake resistor — this is why Zone 1 torque is capped and gated on wheel speed.

## Conventions

- Keep the CAN command/error-bit `#define`s in `ODriveCAN.h` in sync with the ODrive v0.5.x CAN protocol
  (not later ODrive firmware versions).
- Any new tunable that needs to survive reboot goes into `DeviceInfo` + gets a case in the `CFG:`/`GET`
  dashboard protocol in both `BLEDashboard.cpp` and `index.html`.
- Safety-relevant thresholds (hysteresis bounds, dead-man sensor range, regen velocity cutoff, watchdog
  timeout) are currently hardcoded in `main.cpp`/`ODriveCAN.cpp` rather than tunable — treat changes to
  these as safety-critical and cross-check against the physics described in `README.md` before altering.
