# 🎯 Updated Robot Workflow - Smart Localization

## Key Changes

✅ **Manual start trigger** - Press ENTER when ready
✅ **No driving around** - Robot stays in place, just rotates
✅ **4-wall triangulation** - Calculates exact position
✅ **Pre-calculated shots** - All 3 hoops calculated once
✅ **Instant lookup** - No recalculation during game

---

## 🚀 New Startup Sequence

### At Competition Table

```bash
# 1. SSH into RPI
ssh pi@basketballbot

# 2. Navigate to code
cd ~/bot_test

# 3. Start the program
python3 main.py
```

**SCREEN OUTPUT:**
```
============================================================
BASKETBALL ROBOT - BEVO MADNESS
============================================================

[INIT] Connecting to A-Star...
[INIT] ✓ Connected to A-Star
[INIT] Initializing state machine...
[INIT] ✓ State machine ready

============================================================
READY TO START
============================================================

Place robot in starting zone, then press ENTER to begin...
(or type 'q' to quit)

>>> _
```

### You Now:
1. **Place robot in starting zone** (any position, any orientation)
2. **Load 10 balls** into magazine
3. **Press ENTER** when ready
4. **Walk away!**

---

## 🤖 Robot's New Action Sequence

### Phase 1: Scanning (3.6 seconds)

**ROBOT:**
```
"Starting 360° scan to map the arena..."
```

- Rotates in place (drive wheels: left reverse, right forward)
- Takes 72 ultrasonic readings (every 5°)
- Records distance to walls at all angles

**SCREEN OUTPUT:**
```
[LOCALIZE] Starting 360° arena scan for triangulation...
  0°: 48.2"
  90°: 52.1"
  180°: 38.7"
  270°: 55.8"
```

---

### Phase 2: Position Calculation (1.5 seconds)

**ROBOT:**
```
"Analyzing scan data..."
"I see 4 walls - calculating my exact position..."
```

**Actions:**
1. Extract 4 wall distances (East, West, North, South)
2. Calculate position: `x = 72 - dist_to_backboard`, `y = dist_from_left`
3. Calculate distances to ALL 3 hoops:
   - `distance_to_left = sqrt((72-x)² + (15-y)²)`
   - `distance_to_center = sqrt((72-x)² + (36-y)²)`
   - `distance_to_right = sqrt((72-x)² + (57-y)²)`
4. Look up RPM for each distance from table
5. Calculate lazy susan angles for each hoop

**SCREEN OUTPUT:**
```
[LOCALIZE] Detected walls:
  EAST: 45° @ 48.5" (backboard)
  NORTH: 135° @ 52.1"
  WEST: 225° @ 23.5"
  SOUTH: 315° @ 19.8"

[LOCALIZE] Calculated position: (23.5", 52.1")

[LOCALIZE] Pre-calculated shot parameters:
  LEFT: 54.3" @ 6500 RPM, angle -18.2°
  CENTER: 58.7" @ 7000 RPM, angle 0.0°
  RIGHT: 47.2" @ 5800 RPM, angle +15.4°
```

---

### Phase 3: Align to Center (2 seconds)

**ROBOT:**
```
"Rotating to face center hoop..."
```

- If center angle > 15°: Use drive wheels for coarse rotation
- If center angle < 15°: Lazy susan will handle it during shooting

**SCREEN OUTPUT:**
```
[LOCALIZE] Rotating 45° to face center hoop...
[LOCALIZE] Aligned! Updated lazy susan angles:
  LEFT: -18.2°
  CENTER: 0.0°
  RIGHT: +15.4°

[LOCALIZE] ✓ Localization complete! Ready to shoot.
```

---

### Phase 4: Hunting (Waiting)

**ROBOT:**
```
"Waiting for first beacon to activate..."
```

- Flywheels spinning at idle (3500 RPM)
- Lazy susan at 0° (facing center)
- IR sensors monitoring continuously
- **NO DRIVING** - stays perfectly still

**SCREEN OUTPUT:**
```
[STATE] SEEK_AND_ORIENT → HUNT_FOR_TARGET (7234ms)
[STATE] Waiting for active beacon...
```

---

### Phase 5: Shooting! (1 second per shot)

**BEACON ACTIVATES: RIGHT**

**ROBOT:**
```
"Right beacon detected!"
"Looking up pre-calculated parameters..."
"Distance: 47.2\", RPM: 5800, Angle: +15.4°"
"Adjusting flywheels... Rotating lazy susan..."
"FIRE!!!"
```

**Actions:**
1. Detect beacon (instant)
2. **Look up pre-calculated** distance, RPM, angle (instant!)
3. Send flywheel command: 5800 RPM
4. Send lazy susan command: +15.4°
5. Wait for both to stabilize (~400ms)
6. SHOOT (300ms)

**SCREEN OUTPUT:**
```
[STATE] Beacon detected: RIGHT
[STATE] Target: RIGHT (PRE-CALCULATED)
  Distance: 47.2"
  RPM: 5800
  Lazy Susan: +15.4°
[STATE] Ready to shoot (stable for 150ms)
[TIMING] Shot #1: 892ms | Avg: 892ms | Target: 1000ms
[STATE] Shot complete! 9 balls remaining
```

---

**NEXT BEACON: CENTER**

**ROBOT:**
```
"Center beacon! Already aligned!"
"Flywheels adjusting to 7000 RPM..."
"FIRE!!!"
```

**Faster!** - Lazy susan already at 0° (center), only RPM adjustment needed!

```
[STATE] Beacon detected: CENTER
[STATE] Target: CENTER (PRE-CALCULATED)
  Distance: 58.7"
  RPM: 7000
  Lazy Susan: 0.0°
[TIMING] Shot #2: 687ms | Avg: 789ms | Target: 1000ms ✓✓
```

---

**NEXT BEACON: LEFT**

**ROBOT:**
```
"Left beacon!"
"Lazy susan rotating -18.2°..."
"FIRE!!!"
```

```
[TIMING] Shot #3: 915ms | Avg: 831ms | Target: 1000ms
```

---

**Cycle repeats for all 10 balls!**

---

## 📊 Performance Comparison

### Old Strategy (Drive Around):
```
0-8s:   Localize (scan, rotate, DRIVE forward/back)
8-28s:  Shoot 10 balls
        Each shot: Measure distance, calculate RPM, shoot

Total: ~28 seconds
```

### New Strategy (Stay in Place):
```
0-7s:   Localize (scan, calculate ALL positions, rotate)
7-17s:  Shoot 10 balls
        Each shot: LOOKUP pre-calculated params, shoot

Total: ~17 seconds (11 seconds faster!)
```

**Benefits:**
- ✅ No driving complexity
- ✅ No position drift
- ✅ Faster shots (no distance measurement)
- ✅ More accurate (exact position known)
- ✅ Pre-calculated means fewer errors

---

## 🎯 What You See During Competition

```
YOU: python3 main.py
     [Place robot in zone, press ENTER]

ROBOT: *Starts spinning in place, scanning*
       (3.6 seconds)

ROBOT: *Stops spinning*
       "Position: (23\", 52\"), All shots calculated!"

ROBOT: *Rotates to face center*
       "Ready!"

BEACON: *LEFT activates*

ROBOT: *Lazy susan rotates left*
       *THWACK!* Ball launches

BEACON: *CENTER activates*

ROBOT: *Lazy susan centers*
       *THWACK!* Ball launches

...continues until all 10 balls shot...

ROBOT: *Flywheels wind down*
       "Game complete! 7/10 shots made!"

YOU: "Nice! 🎉"
```

---

## 🔧 Calibration Tips

### 1. Test Rotation Speed
The robot rotation during scan needs to be smooth and consistent.

```python
# In config.py, adjust:
ROTATION_SPEED = 100  # Start here, tune if needed
```

Robot should complete 360° in ~4 seconds during scan.

### 2. Verify Position Calculation
Place robot at known position, run localization, check calculated position matches.

```python
# Expected at position (30", 40"):
# Dist to backboard = 72 - 30 = 42"
# Dist from left = 40"
```

### 3. Calibrate RPM Table
The pre-calculated distances will use your RPM lookup table:

```python
# config.py
RPM_LOOKUP_TABLE = {
    24: 4000,  # Test and adjust these!
    36: 5000,
    48: 6000,
    60: 7000
}
```

Test from various positions, verify shots land accurately.

---

## 🐛 Troubleshooting

**Problem: Position calculation is wrong**
- Check ultrasonic readings are accurate
- Verify arena dimensions in config.py match actual court
- Ensure all 4 walls are detected during scan

**Problem: Robot rotates wrong direction**
- Check drive motor directions
- Swap motor polarities if needed
- Adjust rotation logic in code

**Problem: Pre-calculated angles are off**
- Verify basket positions in config.py
- Check coordinate system (x, y) matches your setup
- Robot's "forward" direction matters!

**Problem: Shots still missing**
- RPM lookup table needs calibration
- Test each distance manually first
- Shooter height must be 18" (same as hoop)

---

**Ready to dominate! Your robot is now a positioning expert! 🤖🏀🎯**
