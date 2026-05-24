[README_MacroRail_TMC2209_UltraSmooth.md](https://github.com/user-attachments/files/28199819/README_MacroRail_TMC2209_UltraSmooth.md)
# ESP32-S3 TMC2209 Ultra-Smooth Macro Rail Controller

Arduino sketch for an ESP32-S3 based macro photography rail using TMC2209 stepper drivers, a browser-based control panel, joystick jogging, smooth motion ramps, and Nikon IR shutter triggering.

This project is designed for focus stacking, microscope objective macro work, and slow precision camera rail movement where vibration must be minimized.

> **Security warning:** before uploading this project to GitHub, remove your real Wi-Fi SSID and password from the sketch. Replace them with placeholders or move them to a separate ignored secrets file.

---

## Main Features

- ESP32-S3 web control panel over Wi-Fi
- X-axis stepper control with optional Z-axis support
- TMC2209 STEP/DIR driver control
- Ultra-smooth ramped movement using a smootherstep/S-curve style profile
- Slow photo mode and faster rail adjustment mode
- Manual movement mode
- Semi-auto photo stacking mode
- Full-auto start-to-end stacking mode
- Nikon IR shutter triggering through a 38 kHz IR output
- Adjustable shutter timing, pre-IR wait, and post-rest delay
- Physical joystick jogging for X and Z movement
- Soft travel limits
- Backlash preload support in full-auto mode
- Persistent microstep/profile settings using ESP32 Preferences storage
- mDNS support: `http://esp32.local/`

---

## Hardware Used

### Controller

- ESP32-S3 development board
- Arduino IDE board target: **ESP32S3 Dev Module**

Recommended Arduino IDE settings:

```text
Board: ESP32S3 Dev Module
USB CDC On Boot: Enabled
USB Mode: Hardware CDC and JTAG
```

### Stepper Drivers

- 1x TMC2209 driver for X axis
- Optional 1x TMC2209 driver for Z axis
- STEP/DIR mode only
- No UART configuration in this version

### Motor / Mechanics

The sketch assumes:

```text
Lead screw travel: 2 mm per revolution
Stepper motor: 200 full steps per revolution
Default microstepping: 1/32
Steps per revolution: 6400
Resolution: 0.3125 µm per step
```

If your lead screw, motor, or microstep setting is different, update these values in the sketch:

```cpp
constexpr float LEAD_MM_REV = 2.0f;
static int   gMicrostep   = 32;
static long  gStepsPerRev = 6400;
static float gUmPerStep   = 0.3125f;
```

---

## Pin Map

### X Axis TMC2209

| Function | ESP32-S3 Pin |
|---|---:|
| STEP | GPIO4 |
| DIR | GPIO5 |
| EN / ENN | GPIO6 |

### Z Axis TMC2209 Optional Driver

| Function | ESP32-S3 Pin |
|---|---:|
| STEP | GPIO7 |
| DIR | GPIO15 |
| EN / ENN | GPIO16 |

### Nikon IR Output

| Function | ESP32-S3 Pin |
|---|---:|
| IR LED driver signal | GPIO18 |

Use a transistor or MOSFET driver for the IR LED. Do **not** drive a strong IR LED directly from the ESP32 pin.

### Joystick

| Joystick Pin | ESP32-S3 Pin |
|---|---:|
| VRx | GPIO1 |
| VRy | GPIO2 |
| SW | GPIO42 |
| VCC | 3.3V |
| GND | GND |

The joystick button is configured as `INPUT_PULLUP`, so the switch should connect GPIO42 to GND when pressed.

---

## TMC2209 Wiring Notes

This sketch uses simple STEP/DIR control. It does not use TMC2209 UART.

Recommended common-module wiring for 1/32 microstepping:

| TMC2209 Pin | Connection |
|---|---|
| VIO / VCC_IO | 3.3V |
| GND | ESP32 GND and motor power GND |
| VM | Motor power supply |
| STEP | ESP32 STEP pin |
| DIR | ESP32 DIR pin |
| EN / ENN | ESP32 EN pin |
| MS1 | 3.3V |
| MS2 | GND |
| SPREAD | GND or open for quiet StealthChop mode |
| PDN_UART | 3.3V in this no-UART beginner version |

Important: the code only changes the microstep value used for distance calculation. The real microstep mode is set by the TMC2209 hardware pins. If the web UI says 1/32 but the driver hardware is wired differently, distance calculations will be wrong.

---

## Wi-Fi Setup

Edit this section before uploading to the ESP32:

```cpp
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* HOSTNAME      = "esp32";
```

After upload, open the Serial Monitor at `115200` baud. The ESP32 will print its local IP address.

Example:

```text
Open this address: http://192.168.x.x
mDNS address: http://esp32.local/
```

Then open the address in a browser on the same Wi-Fi network.

---

## Web Control Panel

The built-in web interface supports:

### Motion Profile

- **Photo floating / slow**: ultra-slow, smooth, low-vibration movement for shooting
- **Fast rail adjustment**: faster movement for positioning the rail

### Modes

#### Manual

Moves the selected axis by a chosen number of steps, then optionally fires the Nikon IR shutter.

#### Semi-Auto

Repeats a photo sequence for a selected number of photos. Between photos, the rail moves by a fixed number of steps.

#### Full Auto

Uses a selected start and end point, then automatically moves through the stack using the selected steps-per-photo value.

Full Auto includes:

- Start point capture
- End point capture
- Steps per photo
- Backlash preload
- Inter-shot gap
- Soft minimum and maximum limits
- Optional IR shutter firing

---

## Motion Behavior

The sketch is tuned to avoid sudden motor kicks.

Key behavior:

- Drivers remain enabled after movement to hold position
- Stop command requests a smooth deceleration instead of instantly cutting motor power
- Direction changes decelerate before reversing
- Joystick movement is filtered and ramped
- Slow profile is intentionally conservative for macro photography

The TMC2209 EN/ENN pin is active-low:

```text
LOW  = driver enabled
HIGH = driver disabled
```

---

## IR Shutter Behavior

The sketch generates a Nikon-style IR shutter signal using a 38 kHz carrier.

Timing options in the web UI:

- Pre-IR wait
- Shutter wait
- Post-rest wait

Default values:

```text
Pre-IR wait: 1.0 second
Shutter wait: 1.0 second
Post-rest: 1.0 second
```

For best results, point the IR LED toward the camera IR receiver and use a proper transistor/MOSFET driver circuit for stronger range.

---

## Installation

1. Install the ESP32 board package in Arduino IDE.
2. Select **ESP32S3 Dev Module**.
3. Enable **USB CDC On Boot**.
4. Open `MacroRail_TMC2209_UltraSmooth.ino`.
5. Replace the Wi-Fi SSID and password.
6. Connect the hardware according to the pin map.
7. Upload the sketch.
8. Open Serial Monitor at `115200` baud.
9. Open the printed IP address in your browser.

No external Arduino libraries are required beyond the ESP32 Arduino core libraries used by the sketch:

```cpp
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
```

---

## Safety Notes

- Do not power motors directly from the ESP32.
- Use a separate motor power supply sized for your stepper motors.
- Connect all grounds together: ESP32 GND, motor driver GND, and power supply GND.
- Set TMC2209 current limit correctly before running the motor.
- Add physical limit switches in future versions if the rail can crash mechanically.
- This sketch currently uses software soft limits, not physical homing switches.
- Keep fingers, cables, and camera gear clear of the moving rail.

---

## Current Limitations

- No TMC2209 UART configuration
- No physical limit switch or homing support in this version
- No OLED/TFT display support in this version
- Wi-Fi credentials are stored directly inside the sketch unless manually changed
- Microstep selection in the web UI affects distance calculation only; hardware pins must match

---

## Recommended Future Improvements

- Move Wi-Fi credentials to a separate `secrets.h` file and add it to `.gitignore`
- Add physical home/end limit switches
- Add homing routine
- Add emergency stop input
- Add saved presets for different lenses/objectives
- Add camera brand selection for other IR protocols
- Add UART control for TMC2209 current, microstepping, and diagnostics
- Add LCD/TFT display support for offline use

---

## License

Add your preferred license here. For open-source GitHub projects, common choices are:

- MIT License
- Apache 2.0 License
- GPLv3 License

If you are not sure, MIT is usually the simplest choice for small hardware/software projects.
