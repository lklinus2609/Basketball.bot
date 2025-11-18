# Basketball Robot - Bevo Madness
**ME 348E/392Q Advanced Mechatronics - Fall 2025**

Complete autonomous basketball robot with 1-second shot cycle targeting.

---

## System Overview

### Architecture
```
┌─────────────────────────────────────────────────────────────┐
│                    Raspberry Pi (RPI)                        │
│  - High-level state machine                                  │
│  - Shot calculations (distance → RPM)                        │
│  - Beacon detection logic                                    │
│  - Game strategy                                             │
└──────────────────────┬───────────────────────────────────────┘
                       │ Serial (115200 baud)
                       │ Non-blocking communication
┌──────────────────────┴───────────────────────────────────────┐
│                 Pololu A-Star (Arduino)                       │
│  - Dual flywheel PID control                                 │
│  - Lazy susan stepper control                                │
│  - Loading mechanism stepper                                 │
│  - Sensor reading (IR, ultrasonic, encoders)                 │
│  - Real-time motor control                                   │
└───────────────────────────────────────────────────────────────┘
```

### Hardware Components
- **Raspberry Pi**: High-level control
- **Pololu A-Star**: Low-level motor control
- **2x Flywheel Motors**: 25D 12V HP motors (Pololu), 40mm diameter flywheels at 15°
- **L298N**: Dual H-bridge for flywheel control
- **2x Encoders**: Flywheel speed feedback
- **2x NEMA17 Steppers**: Lazy susan rotation + loading mechanism
- **2x DRV8825**: Stepper drivers
- **3x TSOP34156**: IR receivers for 56kHz beacon detection (left/center/right)
- **Ultrasonic Sensor**: Distance measurement (HC-SR04 compatible)
- **Drive Wheels**: With encoders for odometry

---

## File Structure

```
bot_test/
├── RPI Python Code (Raspberry Pi)
│   ├── main.py              # Main entry point - run this!
│   ├── state_machine.py     # Main state machine logic
│   ├── communication.py     # Serial communication with A-Star
│   ├── ballistics.py        # RPM calculations and lookup table
│   └── config.py            # Configuration parameters (TUNE HERE!)
│
└── A-Star Arduino Code
    ├── astar_controller.ino         # Main Arduino sketch
    ├── config.h                     # Pin definitions and constants
    ├── communication.h/.cpp         # Serial protocol implementation
    ├── flywheel_control.h/.cpp      # Dual PID controller
    └── stepper_and_sensors.h/.cpp   # Steppers and sensor reading
```

---

## Quick Start Guide

### 1. Hardware Setup

#### Pin Connections (A-Star)

**L298N Flywheel Control:**
- Left Flywheel PWM → Pin 5
- Left IN1 → Pin 6
- Left IN2 → Pin 7
- Right Flywheel PWM → Pin 9
- Right IN1 → Pin 10
- Right IN2 → Pin 11

**Flywheel Encoders:**
- Left Encoder A → Pin 2 (INT0)
- Left Encoder B → Pin 4
- Right Encoder A → Pin 3 (INT1)
- Right Encoder B → Pin 8

**DRV8825 Lazy Susan:**
- STEP → Pin A0
- DIR → Pin A1
- ENABLE → Pin A2 (connect to GND or use LOW to enable)

**DRV8825 Loader:**
- STEP → Pin A3
- DIR → Pin A4
- ENABLE → Pin A5

**IR Sensors (TSOP34156):**
- Left → Pin 12
- Center → Pin 13
- Right → Pin 14
(Note: Active LOW - outputs LOW when 56kHz detected)

**Ultrasonic Sensor:**
- TRIG → Pin 15
- ECHO → Pin 16

#### Power Requirements
- **12V**: Flywheel motors (L298N), stepper motors (DRV8825)
- **5V**: A-Star, encoders, sensors
- **Power**: Raspberry Pi via USB or dedicated 5V supply

### 2. Software Setup

#### A-Star (Arduino)

1. **Install Arduino IDE** (if not already installed)

2. **Install required libraries:**
   ```
   Tools → Manage Libraries
   Search and install: "AccelStepper by Mike McCauley"
   ```

3. **Open sketch:**
   ```
   File → Open → astar_controller/astar_controller.ino
   ```

4. **Adjust pin assignments** in `config.h` if your wiring differs

5. **Upload to A-Star:**
   ```
   Tools → Board → Select your A-Star board
   Tools → Port → Select correct port
   Sketch → Upload
   ```

#### Raspberry Pi (Python)

1. **Install Python dependencies:**
   ```bash
   pip3 install pyserial
   ```

2. **Configure serial port** in `config.py`:
   ```python
   SERIAL_PORT = '/dev/ttyAMA0'  # Or '/dev/ttyUSB0' if using USB
   ```

3. **Enable serial on RPI:**
   ```bash
   sudo raspi-config
   # Interface Options → Serial Port
   # Login shell over serial: NO
   # Serial port hardware enabled: YES
   ```

4. **Test connection:**
   ```bash
   python3 communication.py  # Run communication test
   ```

5. **Run the robot:**
   ```bash
   python3 main.py
   ```

---

## Configuration & Tuning

### Critical Parameters (config.py)

#### 1. RPM Lookup Table
```python
RPM_LOOKUP_TABLE = {
    24: 4000,   # Close shot
    36: 5000,   # Medium
    48: 6000,   # Far shot
    60: 7000    # Max range
}
```

**Calibration Procedure:**
1. Place robot at known distance (e.g., 36")
2. Manually set RPM and test shots
3. Adjust RPM until shots are consistent
4. Record distance → RPM mapping
5. Repeat for multiple distances
6. Update lookup table

#### 2. Basket Angles
```python
BASKET_ANGLES = {
    'CENTER': 0.0,
    'LEFT': -12.0,   # Adjust based on testing
    'RIGHT': 12.0
}
```

**Calibration:**
1. Place robot at typical shooting position
2. Manually aim lazy susan at each basket
3. Measure angle with protractor or encoder feedback
4. Update angles in config

#### 3. PID Tuning (config.h)
```cpp
#define FLYWHEEL_KP  0.5
#define FLYWHEEL_KI  0.1
#define FLYWHEEL_KD  0.05
```

**Tuning Procedure (Ziegler-Nichols):**
1. Set Ki=0, Kd=0
2. Increase Kp until oscillation occurs (note Kp_critical)
3. Kp = 0.6 * Kp_critical
4. Ki = 2 * Kp / Period_oscillation
5. Kd = Kp * Period_oscillation / 8
6. Fine-tune by testing

#### 4. Encoder PPR
```cpp
#define ENCODER_PPR  360  // Pulses per revolution - MEASURE THIS!
```

**How to measure:**
1. Rotate flywheel exactly 10 revolutions by hand
2. Read encoder count
3. PPR = count / 10

---

## State Machine Flow

```
┌────────────────┐
│      INIT      │  Initialize hardware, spin up to idle RPM
└───────┬────────┘
        ↓
┌────────────────┐
│ HUNT_FOR_TARGET│  Monitor IR sensors, wait for active beacon
└───────┬────────┘
        ↓ (Beacon detected)
┌────────────────┐
│ CALCULATE_SHOT │  Measure distance, lookup RPM, calc angle
└───────┬────────┘  (~50ms)
        ↓
┌────────────────┐
│ALIGN_AND_SPINUP│  Rotate lazy susan + adjust flywheel RPM
└───────┬────────┘  (parallel, ~400ms)
        ↓ (Ready to shoot)
┌────────────────┐
│ EXECUTE_SHOT   │  Fire loading mechanism (90° rotation)
└───────┬────────┘  (~300ms)
        ↓
        └─→ Back to HUNT_FOR_TARGET (if balls remain)
            or END_GAME (if no balls left)

TOTAL CYCLE: ~1000ms (1 second)
```

---

## Timing Budget (1-Second Target)

| Phase | Target Duration | Critical Actions |
|-------|-----------------|------------------|
| Beacon Detection | 0-50ms | ID active beacon, measure distance, lookup RPM |
| Command Send | 50ms | Send RPM + angle commands (parallel) |
| Alignment | 50-450ms | **Concurrent**: Lazy susan rotate + flywheel adjust |
| Stabilization | 450-650ms | Verify alignment + RPM stable |
| Shoot | 650-950ms | Loading mechanism 90° rotation |
| Reset | 950-1000ms | Mechanism ready, next ball loaded |

**Optimization Tips:**
- Keep lazy susan at center when idle (minimizes rotation distance)
- Flywheels run continuously (only adjust speed, never stop)
- Use predictive shooting (shoot 50ms early to account for ball drop)
- Timeout fallback: shoot anyway after 600ms even if not perfect

---

## Troubleshooting

### Robot Not Shooting Fast Enough
1. Check `ALIGN_SPINUP_TIMEOUT` - increase if needed
2. Reduce `STABILIZATION_TIME` - try 50ms instead of 100ms
3. Increase stepper acceleration in `config.h`
4. Use `PREDICTIVE_SHOOT_ADVANCE` to shoot earlier

### Flywheels Not Reaching Target RPM
1. Check PID tuning - may need higher Kp
2. Verify encoder connections (should see pulses)
3. Check power supply - 12V should be stable
4. Measure actual `ENCODER_PPR` - incorrect value = wrong RPM

### Shots Missing Target
1. Re-calibrate RPM lookup table empirically
2. Check lazy susan angles - verify with actual measurements
3. Verify shooter height = 18" (same as hoop)
4. Check flywheel angle = 15°
5. Test ultrasonic accuracy - compare to ruler

### Communication Errors
1. Check baud rate matches (115200)
2. Verify serial port in `config.py`
3. Check wiring: RPI TX → A-Star RX, RPI RX → A-Star TX
4. Test with: `python3 communication.py`

### Steppers Not Moving
1. Verify DRV8825 ENABLE is LOW (active low)
2. Check power supply to drivers (12V)
3. Verify microstepping jumpers on DRV8825
4. Check step/dir pins are correct

---

## Testing Procedures

### 1. Component Tests

**Flywheel Test:**
```bash
# On A-Star (Arduino Serial Monitor):
# Send command via RPI or manual test
python3 -c "from communication import *; c=AStarCommunication(); c.connect(); c.set_flywheel_rpm(5000); import time; time.sleep(3)"
```

**Lazy Susan Test:**
```bash
python3 -c "from communication import *; c=AStarCommunication(); c.connect(); c.rotate_lazy_susan(15); import time; time.sleep(2)"
```

**Loader Test:**
```bash
python3 -c "from communication import *; c=AStarCommunication(); c.connect(); c.shoot(); import time; time.sleep(1)"
```

### 2. Ballistics Test
```bash
python3 ballistics.py  # Runs test of RPM calculations
```

### 3. Full System Test
```bash
python3 main.py
# Robot should:
# 1. Initialize (flywheels spin to idle)
# 2. Wait for IR beacon
# 3. Detect beacon, calculate shot
# 4. Align and shoot
# 5. Repeat until all balls shot
```

---

## Competition Day Checklist

- [ ] Charge all batteries fully
- [ ] Verify all wiring connections secure
- [ ] Upload latest code to A-Star
- [ ] Copy latest Python code to RPI
- [ ] Test flywheels spin smoothly
- [ ] Test lazy susan full rotation
- [ ] Test loading mechanism fires reliably
- [ ] Verify IR sensors detect beacons
- [ ] Calibrate RPM lookup table on actual court
- [ ] Test full autonomous run (10 balls)
- [ ] Verify 1-second shot timing
- [ ] Have spare batteries ready
- [ ] Bring tools for quick adjustments

---

## Performance Metrics

### Target Specifications
- **Shot cycle time**: <1000ms (beacon detection → ball launch)
- **Flywheel RPM accuracy**: ±2% of target
- **Lazy susan alignment**: ±3° of target
- **Shot success rate**: >70% (practice target)

### Logging & Analysis
Shot timing is automatically logged. Check output:
```
[TIMING] Shot #1: 987ms | Avg: 987ms | Target: 1000ms
[TIMING] Shot #2: 1032ms | Avg: 1010ms | Target: 1000ms
  ⚠️  WARNING: Exceeding 1-second target!
```

---

## Advanced Tuning

### Predictive Shooting
Shoot slightly before perfect conditions to account for:
- Ball drop time from magazine (~50ms)
- Lazy susan settling time
- Control loop latency

Adjust in `config.py`:
```python
PREDICTIVE_SHOOT_ADVANCE = 50  # Shoot 50ms early
```

### Flywheel Speed Caching
Robot remembers last shot RPM. If next shot is similar distance, skips RPM change (saves time).

### Aggressive Timeouts
If alignment takes too long, robot shoots anyway (imperfect shot better than no shot):
```python
ALIGN_SPINUP_TIMEOUT = 600  # Max wait time (ms)
```

---

## Credits

**Course**: ME 348E/392Q Advanced Mechatronics, UT Austin, Fall 2025
**Competition**: Bevo Madness Tournament
**Adapted from**: Ed Carryer, ME210, Stanford University

---

## License

Educational use for ME 348E/392Q students.

---

**Good luck at the competition! 🏀🤖**
