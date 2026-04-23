# 2025 NNHS Electric Vehicle 
## Features

- **Hardware step generation** via FspTimer ISR (50 kHz) — motor speed is independent of `loop()` timing
- **IMU heading feedback** using ICM-20948 gyroscope (direct I2C register access, no library)
- **Differential steering math** — left/right wheels travel proportional arc lengths
- **BLE live tuning** — adjust KP, KD, BIAS, radius, angle, time, and dist correction from a browser dashboard without reflashing
- **Web Bluetooth dashboard** — connect from Chrome, send parameters, monitor heading error and arc progress in real time

---

## Hardware

### Components

| Component | Details |
|-----------|---------|
| Microcontroller | Arduino UNO R4 WiFi |
| Stepper drivers | TMC2209 × 2 (standalone mode) |
| IMU | SparkFun ICM-20948 (I2C, address 0x69) |
| Motors | NEMA 17 stepper motors × 2 |
| Power supply | 12V, 4A minimum (2A per motor) |
| Start button | Momentary pushbutton |

### Physical Parameters

Measure these on your vehicle and update in the sketch:

| Parameter | Variable | Notes |
|-----------|----------|-------|
| Wheel diameter | `WHEEL_DIAMETER_MM` | Outer diameter of drive wheel |
| Effective track width | `TRACK_WIDTH_MM` | **Calibrate experimentally** — not the physical axle width. Start with physical measurement and binary-search until arc angle matches target. |
| Steps per revolution | `STEPS_PER_REV` | 200 for standard NEMA 17 |
| Microstepping | `MICROSTEP` | Must match physical driver pin wiring |

> ⚠️ **Track width is critical.** The effective track width differs from the physical measurement due to wheel contact geometry and slip. Binary-search between physical width and a smaller value until the open-loop arc angle is accurate.

---

## Wiring

### Stepper Drivers (TMC2209 — Standalone Mode)

| Arduino Pin | Left Motor | Right Motor |
|-------------|------------|-------------|
| 9 | LEFT STEP | — |
| 12 | LEFT DIR | — |
| 8 | LEFT EN | — |
| 10 | — | RIGHT STEP |
| 6 | — | RIGHT DIR |
| 7 | — | RIGHT EN |

**TMC2209 pin connections:**

```
VCC_IO → 5V (logic voltage)
GND    → Arduino GND and power supply (−)
VM     → 12V power supply (+)
STEP   → Arduino STEP pin
DIR    → Arduino DIR pin
ENN    → Arduino EN pin  (active LOW — same as A4988/DRV8825)
PDN_UART → leave floating (not used in standalone mode)
MS1    → see microstepping table below
MS2    → see microstepping table below
SPREAD → GND for StealthChop (silent), 5V for SpreadCycle (more torque)
```

> ⚠️ **Do NOT connect UART (PDN_UART) to Arduino TX/RX unless you intend to use UART mode.** In standalone mode, leave PDN_UART floating or pull it to GND through a 100kΩ resistor.

**Microstepping — MS1 / MS2 pins:**

| Microsteps | MS1 | MS2 |
|------------|-----|-----|
| 8 (default, no wiring needed) | NC / GND | NC / GND |
| 2 | 5V | GND |
| 4 | GND | 5V |
| 16 | 5V | 5V |

> The TMC2209 defaults to **8 microsteps** when MS1 and MS2 are left unconnected (internal pull-down). Set `MICROSTEP = 8` in the sketch to match.

**Current setting via VREF:**

TMC2209 sets motor current via the VREF potentiometer on the driver board:

```
I_RMS = VREF / 1.41 / 0.11   (for typical 0.11Ω sense resistor boards)
```

| Target I_RMS | VREF |
|-------------|------|
| 0.5A | 0.78V |
| 0.8A | 1.24V |
| 1.0A | 1.55V |
| 1.2A | 1.86V |

Measure VREF between the potentiometer wiper and GND while the driver is powered.


> ⚠️ Add a **1000 µF electrolytic** capacitor across VM and GND on each driver, as close to the pins as possible, to absorb motor startup current spikes.

### ICM-20948 IMU

| ICM-20948 Pin | Arduino Pin |
|---------------|-------------|
| VCC | 3.3V |
| GND | GND |
| SDA | SDA (A4) |
| SCL | SCL (A5) |
| AD0 | 3.3V (sets I2C address to 0x69) |

### Start Button

| Button Pin | Arduino Pin |
|------------|-------------|
| One side | Pin 2 |
| Other side | 5V |

Internal pull-down assumed. Button reads HIGH when pressed.

---

## Arc Geometry

The vehicle drives a **right-turning** circular arc.

```
Left wheel  = outer wheel → longer arc = (R + W/2) × θ
Right wheel = inner wheel → shorter arc = (R - W/2) × θ

where:
  R = arc radius (mm)
  W = effective track width (mm)
  θ = arc angle (radians)
```

To achieve a chord length of 7000 mm:
```
chord = 2 × R × sin(θ/2)
7000  = 2 × 8056.25 × sin(25.75°)  ✓
```

---

## Software

### Files

| File | Description |
|------|-------------|
| `CurveRouteStepper.ino` | Arduino sketch |
| `SciolyEV_Dashboard_v2.html` | Web Bluetooth dashboard (open in Chrome) |
| `CurveRouteClac.py` | Python Route Calculator |

### Route Calculator
Output: `(RADIUS, ARC_ANGLE, STARTING_ANGLE)`
| Output | Description|
|--------|------------|
| `RADIUS` | Arc radius in mm |
| `ARC_ANGLE` | Arc angle in degrees |
| `STARTING_ANGLE` | The angle between the vehicle's midline and the imagined line between the start point and the end point in degrees |

### Arc Parameters (tunable via BLE)

| BLE Command | Variable | Description |
|-------------|----------|-------------|
| `RADIUS:1000` | `rt_ARC_RADIUS_MM` | Arc radius in mm |
| `ANGLE:90` | `rt_ARC_ANGLE_DEG` | Arc angle in degrees |
| `TIME:8000` | `rt_TARGET_TIME_MS` | Time to complete arc in ms |
| `KP:12` | `rt_KP_HEADING` | Proportional heading gain |
| `KD:0.5` | `rt_KD_HEADING` | Derivative heading gain (dampens oscillation) |
| `BIAS:0` | `rt_BIAS` | Speed trim between left and right motors |
| `DIST:0` | `rt_DIST_CORRECTION_MM` | Add/subtract distance from both wheels |
| `RESET:1` | — | Stop vehicle and wait for button press |

### IMU Heading Correction

```
expectedHeading = -(arcProgress × ARC_ANGLE_DEG)   // negative = right turn
headingError    = actualGyroHeading - expectedHeading
correction      = KP × headingError + KD × d(error)/dt
leftSpeed       = baseSpeed + correction + BIAS
rightSpeed      = baseSpeed - correction - BIAS
```

Positive error = under-turning → speeds up left (outer) wheel  
Negative error = over-turning → speeds up right (inner) wheel

---

## Setup & Usage

### 1. Arduino Setup

1. Install **ArduinoBLE** library from Library Manager
2. Set physical parameters at the top of the sketch
3. Upload `SciolyEV_Arc_v2.ino` to UNO R4 WiFi
4. Open Serial Monitor at 115200 baud — wait for:
   ```
   [6] BLE OK - advertising as 'SciolyEV'
   ```

### 2. Calculate Route
1. Open`CurveRouteClac.py` with VSCode or another IDE.
2. Run the program and enter the desired chord length.
3. Enter the parameters in the Dashboard

### 3. Dashboard Setup

1. Open `SciolyEV_Dashboard_v2.html` in **Google Chrome** (Web Bluetooth requires Chrome)
2. Click **Connect** and select `SciolyEV`
3. Set desired parameters in the tuning panel
4. Click **↑ Send All Parameters**
   
### 4. Running the Vehicle

1. Place vehicle at start position with the correct `STARTING_ANGLE`, keep still during gyro calibration (~10 seconds at boot)
2. Connect dashboard, send parameters
3. Press the start button on the vehicle
4. Vehicle drives the arc and stops automatically
5. Dashboard shows heading error, motor speeds, and arc progress in real time

### 5. Between Runs

1. Click **↺ Reset & Send New Parameters** on the dashboard
2. Adjust parameters as needed
3. Press the start button on the vehicle again — no USB cable required
