# IRscan - IR Beacon Panning Test

Test scripts for IR beacon detection with panning motion.

## Files

- **IRscan.py** - Python script with panning logic
- **IRscan.ino** - Arduino sketch for motor control

## Behavior

1. **Panning Motion**: Robot rotates back-and-forth within ~90° range (±45° from center)
   - Pans for 3 seconds in each direction
   - Automatically reverses direction

2. **Beacon Detection**: Uses same hysteresis filtering as IRmove
   - ≥20% detection (≥4/20 samples) → STOP
   - <10% detection (<2/20 samples) → RESUME

3. **Stop Hold**: When beacon detected, holds position for minimum 0.5 seconds before resuming scan

## Setup

1. Upload `IRscan.ino` to A-Star (Arduino IDE)
2. Run `python3 IRscan.py` on Raspberry Pi

## Configuration

Edit these constants in `IRscan.py`:

```python
PAN_SPEED = 75           # Motor speed (0-400)
PAN_DURATION = 3.0       # Seconds per direction
STOP_DURATION = 0.5      # Stop hold time (seconds)
```

## Testing

1. Run script - robot should pan left/right continuously
2. Turn on IR beacon - robot should stop within 0.2s
3. Verify it holds for at least 0.5s
4. Turn off beacon - robot resumes panning after hold time
