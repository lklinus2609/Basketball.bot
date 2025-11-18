# Localization System Documentation

## Overview

The robot performs automatic localization at startup since it's placed in a **random position and orientation** within the starting zone. The localization phase runs before shooting begins.

---

## Localization Strategy

### Phase 1: 360° Scan (0-4 seconds)
**Goal**: Find the backboard

**Method**:
1. Robot rotates in place (differential drive: left=-speed, right=+speed)
2. Takes 36 ultrasonic distance samples (every 10°)
3. Records angle and distance for each sample
4. Filters out invalid readings (< 12" or > 72")

**Output**: Array of (angle, distance) pairs

```python
# Example scan data
scan_readings = [
    (0°, 48"),     # Some direction
    (10°, 45"),
    (20°, 42"),
    ...
    (180°, 24"),   # Backboard! (minimum distance)
    ...
]
```

### Phase 2: Orient to Backboard (4-5 seconds)
**Goal**: Face the hoops

**Method**:
1. Find minimum distance from scan → backboard location
2. Calculate rotation angle needed
3. Rotate robot to face backboard (timed rotation)
4. Stop with robot facing hoops

**Output**: Robot oriented toward backboard

### Phase 3: Position at Optimal Distance (5-8 seconds)
**Goal**: Move to ideal shooting distance

**Method**:
1. Measure current distance to backboard (ultrasonic)
2. Compare to target: `OPTIMAL_SHOOTING_DISTANCE = 36"`
3. Drive forward if too far, backward if too close
4. Stop when within tolerance (±3")
5. Verify IR beacons are visible

**Output**: Robot positioned and ready to shoot

---

## Configuration Parameters

In `config.py`:

```python
# Localization Parameters
OPTIMAL_SHOOTING_DISTANCE = 36    # Target distance (inches)
DISTANCE_TOLERANCE = 3            # ±3 inches acceptable

# Motion speeds
ROTATION_SPEED = 100              # Rotation speed (0-255)
DRIVE_SPEED_SLOW = 80             # Slow approach
DRIVE_SPEED_FAST = 150            # Quick repositioning

# Scan settings
SCAN_SAMPLES = 36                 # 36 samples = 10° resolution
SCAN_DELAY_MS = 100               # 100ms between samples

# Distance filtering
MIN_VALID_DISTANCE = 12           # Ignore < 12"
MAX_VALID_DISTANCE = 72           # Ignore > 72"
```

---

## State Machine Integration

### Updated State Flow

```
┌────────────────┐
│      INIT      │  Initialize hardware (500ms)
└───────┬────────┘
        ↓
┌────────────────┐
│ SEEK_AND_ORIENT│  ⭐ NEW: Localization (5-8 seconds)
│                │  - Scan 360°
│                │  - Find backboard
│                │  - Orient and position
└───────┬────────┘
        ↓
┌────────────────┐
│ HUNT_FOR_TARGET│  Wait for active IR beacon
└───────┬────────┘
        ↓
   [Continue with shooting states...]
```

### Timing Impact

**Total startup time**: ~6-9 seconds
- INIT: 0.5s
- SEEK_AND_ORIENT: 5-8s
- Then ready to shoot!

**Once localized**: Normal 1-second shot cycle resumes

---

## Drive Wheel Control

### Hardware Requirements

**Option 1: Separate Drive Motors** (Recommended)
- 2 DC motors for driving (independent from flywheels)
- Connected via motor driver (L298N or similar)
- A-Star pins: 19-22 (adjust in `config.h`)

**Option 2: Use Flywheel Motors** (If no separate drive)
- Flywheels must stop spinning during localization
- Not ideal but workable
- Modify pin definitions to reuse flywheel driver

### Pin Assignments (A-Star)

In `astar_controller/config.h`:

```cpp
// Drive Wheels
#define DRIVE_LEFT_PWM   19   // PWM for left wheel
#define DRIVE_LEFT_DIR   20   // Direction control
#define DRIVE_RIGHT_PWM  21   // PWM for right wheel
#define DRIVE_RIGHT_DIR  22   // Direction control
```

**Adjust these pins** based on your motor driver connections!

### Control Commands

```python
# Python (RPI)
comm.set_drive(left_speed, right_speed)

# Examples:
comm.set_drive(100, 100)      # Forward at speed 100
comm.set_drive(-100, -100)    # Backward at speed 100
comm.set_drive(-100, 100)     # Rotate clockwise (in place)
comm.set_drive(100, -100)     # Rotate counter-clockwise
comm.set_drive(0, 0)          # Stop
```

Speed range: -255 (full reverse) to +255 (full forward)

---

## Fallback Behavior

### If Scan Fails
- No valid distance readings → Skip to hunting anyway
- Robot trusts it's in "good enough" position
- Rely on lazy susan for fine aiming

### If Timeout Occurs
- Max 8 seconds for full localization
- If not positioned perfectly → proceed anyway
- Better to start shooting than waste time

### If IR Beacons Not Visible
- Wait up to 3 seconds after positioning
- Then proceed to hunting (will detect when beacon activates)

---

## Calibration Tips

### 1. Test Rotation Speed
```python
# In calibrate.py, test rotation
comm.set_drive(-100, 100)  # Start rotating
time.sleep(3)              # Time for ~360°
comm.set_drive(0, 0)       # Stop
```

Adjust `ROTATION_SPEED` so robot completes 360° in ~3-4 seconds.

### 2. Verify Ultrasonic Scanning
```python
# Monitor distance readings during scan
# Should see minimum when facing backboard
# Typical: 24-48" when facing hoops, 60-72" when facing away
```

### 3. Test Drive Speeds
```python
# Drive forward for 2 seconds, measure distance traveled
comm.set_drive(80, 80)
time.sleep(2)
comm.set_drive(0, 0)
# Measure physical distance moved
```

Adjust `DRIVE_SPEED_SLOW` for controlled positioning.

### 4. Optimal Shooting Distance
- Test shots from various distances (24", 36", 48")
- Find distance with best accuracy
- Update `OPTIMAL_SHOOTING_DISTANCE`

---

## Debugging Localization

### Enable Debug Output

Already enabled in code! You'll see:

```
[LOCALIZE] Starting 360° scan...
  Scan: 0° = 45.2"
  Scan: 60° = 52.1"
  Scan: 120° = 38.7"
  Scan: 180° = 24.5"   ← Minimum = backboard!
  ...
[LOCALIZE] Backboard detected at angle 180°, distance 24.5"
[LOCALIZE] Rotating 180° to face backboard...
[LOCALIZE] Moving forward 12.5" to target...
[LOCALIZE] Positioned at 36.2" from backboard ✓
[LOCALIZE] Localization complete! Beacon visible: CENTER
```

### Common Issues

**Problem: Robot spins but doesn't find backboard**
- Check ultrasonic sensor is working
- Verify distance readings are valid
- Adjust `MIN_VALID_DISTANCE` / `MAX_VALID_DISTANCE`

**Problem: Robot faces wrong direction**
- Check drive motor directions (might be reversed)
- Swap `left_speed` and `right_speed` in rotation command
- Or reverse motor direction pins

**Problem: Robot drives wrong direction**
- Check `DRIVE_LEFT_DIR` and `DRIVE_RIGHT_DIR` pins
- Might need to invert direction logic (HIGH ↔ LOW)

**Problem: Localization takes too long**
- Reduce `SCAN_SAMPLES` (e.g., 18 samples = 20° resolution)
- Increase `ROTATION_SPEED` (but watch for instability)
- Reduce `SCAN_DELAY_MS` (but ensure ultrasonic has time)

---

## Advanced Optimization

### Skip Localization (Competition Mode)
If you **always place robot at same position** during testing:

```python
# In state_machine.py, modify _state_init():
def _state_init(self):
    if self.time_in_state() > 500:
        # SKIP localization - go straight to hunting
        self.transition_to(State.HUNT_FOR_TARGET)
```

Saves 5-8 seconds at startup!

### Faster Scanning
Reduce samples for quicker scan:
```python
SCAN_SAMPLES = 18     # 20° resolution (half the samples)
SCAN_DELAY_MS = 50    # Faster scanning
```

Total scan time: 18 × 50ms = 0.9 seconds (vs 3.6 seconds)

### Use Odometry (Advanced)
If drive wheels have encoders:
- Track actual distance traveled
- More accurate positioning
- Implementation: Read encoders, integrate to get position

---

## Integration with Competition Strategy

### Startup Sequence

```
0s:   Game starts, robot placed randomly
0.5s: Hardware initialized
1s:   Scanning begins (robot rotating)
4s:   Scan complete, backboard identified
5s:   Oriented toward backboard
6s:   Positioned at 36"
7s:   IR beacon activates
8s:   First shot fired!
```

**Time to first shot: ~8 seconds** (including localization)

### During Game
- Localization runs ONCE at startup only
- After that, robot stays in position
- Uses lazy susan for aiming (no driving)
- Completes all 10 shots in ~20 seconds total

**Total game time: ~28 seconds** (8s localization + 20s shooting)

---

## Testing Checklist

- [ ] Verify ultrasonic reads distances correctly
- [ ] Test 360° scan completes in ~4 seconds
- [ ] Confirm backboard is detected (minimum distance)
- [ ] Check robot rotates to face backboard
- [ ] Verify robot drives forward/backward correctly
- [ ] Test positioning accuracy (±3" of target)
- [ ] Confirm IR beacons visible after positioning
- [ ] Run full localization → shooting sequence
- [ ] Test from different starting positions
- [ ] Test different starting orientations

---

**Localization complete!** The robot now handles random placement automatically. 🎯🤖
