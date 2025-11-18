# Basketball Robot - Bevo Madness

Autonomous basketball robot for ME 348E/392Q Fall 2025 competition.

---

## Hardware

- **RPI**: High-level control, state machine
- **A-Star**: Motor control, sensors, PID
- **2x 25D 12V HP motors**: Flywheels (40mm, 15° angle) via L298N
- **2x NEMA17**: Lazy susan + loader via DRV8825
- **3x TSOP34156**: IR sensors (56kHz, active-low)
- **HC-SR04**: Ultrasonic distance sensor
- **Drive wheels**: For localization rotation

---

## Quick Start

### Upload A-Star Code
```
1. Open Arduino IDE
2. Install library: AccelStepper
3. Open: astar_controller/astar_controller.ino
4. Upload to A-Star
```

### Setup RPI
```bash
pip3 install pyserial
# Edit config.py: Set SERIAL_PORT = '/dev/ttyAMA0'
```

### Run
```bash
python3 main.py
# Press ENTER when robot is placed
```

---

## How It Works

### Localization (7 seconds)
1. **Scan 360°** - Ultrasonic reads distances to all 4 walls
2. **Calculate position** - Triangulate (x,y) from wall distances
3. **Pre-calculate shots** - Compute distance, RPM, angle for all 3 hoops
4. **Rotate to center** - Face middle hoop
5. **Ready** - Stays in place, waits for beacons

### Shooting (<1 second per shot)
1. **Beacon detected** - IR sensor goes LOW
2. **Lookup parameters** - Use pre-calculated distance/RPM/angle
3. **Rotate lazy susan** - Aim at target hoop
4. **Adjust flywheels** - Set to target RPM (PID controlled)
5. **Fire** - Loading mechanism 90° rotation
6. **Return to center** - Lazy susan back to 0°

---

## Key Configuration

### `config.py` (RPI)

**Arena dimensions:**
```python
ARENA_WIDTH = 72.0
ARENA_LENGTH = 72.0
BASKET_POSITIONS = {
    'LEFT': (72.0, 15.0),
    'CENTER': (72.0, 36.0),
    'RIGHT': (72.0, 57.0)
}
```

**Calibration parameters:**
```python
RPM_LOOKUP_TABLE = {
    24: 4000,  # Calibrate empirically
    36: 5000,
    48: 6000,
    60: 7000
}

ROTATION_SPEED = 100        # Scan rotation speed
SCAN_SAMPLES = 72           # 360° / 72 = 5° resolution
RETURN_TO_CENTER_AFTER_SHOT = True
```

### `config.h` (A-Star)

**Control parameters:**
```cpp
// PID for flywheels
#define FLYWHEEL_KP  0.5
#define FLYWHEEL_KI  0.1
#define FLYWHEEL_KD  0.05

// Encoder pulses per revolution
#define ENCODER_PPR  360

// Stepper speeds
#define LAZY_SUSAN_MAX_SPEED    800
#define LOADER_MAX_SPEED        1000
```

---

## Pin Assignments (A-Star)

**Flywheels (L298N):**
- Left: PWM=5, IN1=6, IN2=7
- Right: PWM=9, IN1=10, IN2=11
- Encoders: Left=2/4, Right=3/8

**Steppers (DRV8825):**
- Lazy Susan: STEP=A0, DIR=A1, EN=A2
- Loader: STEP=A3, DIR=A4, EN=A5

**Sensors:**
- IR: Left=12, Center=13, Right=14
- Ultrasonic: TRIG=15, ECHO=16

**Drive wheels:**
- Left: PWM=19, DIR=20
- Right: PWM=21, DIR=22

---

## Calibration

### 1. Encoder PPR
Rotate flywheel 10 revolutions, count pulses, divide by 10.

### 2. PID Tuning
```bash
python3 calibrate.py
# Test RPMs, adjust Kp/Ki/Kd until stable
```

### 3. RPM Lookup Table
Place robot at known distances (24", 36", 48", 60"), test RPMs until shots land consistently. Update `RPM_LOOKUP_TABLE`.

### 4. Test Localization
```bash
python3 main.py
# Check calculated position matches actual position
```

---

## Troubleshooting

**Position calculation wrong:**
- Verify ultrasonic readings are accurate
- Check arena dimensions in config.py

**Shots missing:**
- Recalibrate RPM lookup table
- Verify shooter height = 18"
- Check flywheel angle = 15°

**Flywheels not reaching RPM:**
- Verify ENCODER_PPR is correct
- Tune PID gains
- Check 12V power supply

**Communication errors:**
- Check serial port in config.py
- Verify baud rate = 115200
- Test: `python3 communication.py`

---

## Competition Day

```bash
1. Place robot in starting zone (any position/orientation)
2. Load 10 balls
3. ssh pi@robot-ip
4. cd ~/bot_test
5. python3 main.py
6. Press ENTER
7. Walk away
```

**Expected time:**
- Localization: 7s
- 10 shots: ~10s (avg 1s/shot)
- **Total: ~17s**

---

## Files

- `main.py` - Entry point, manual start trigger
- `state_machine.py` - 7-state FSM, localization, shooting
- `communication.py` - Non-blocking serial to A-Star
- `ballistics.py` - RPM lookup with interpolation
- `config.py` - All tunable parameters
- `calibrate.py` - Interactive testing tool
- `UPDATED_WORKFLOW.md` - Detailed walkthrough
- `LOCALIZATION.md` - Localization documentation

---
