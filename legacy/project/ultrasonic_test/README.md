# Ultrasonic 360° Scan Test

Python interface for controlling the ultrasonic scan test via SSH.

## Setup

1. **Upload Arduino sketch** to A-Star 32U4:
   - Open `ultrasonic_test.ino` in Arduino IDE
   - Upload to the board

2. **Find serial port**:
   ```bash
   ls /dev/ttyACM*
   # or
   ls /dev/ttyUSB*
   ```

## Usage

### Interactive Mode (Default)
```bash
python3 ultrasonic_test.py
```

Commands in interactive mode:
- `s` - Start 360° scan and find furthest wall
- `t` - Test single ultrasonic reading
- `c` - Calibrate rotation timing
- `q` - Quit

### Auto Mode (Single Scan)
```bash
python3 ultrasonic_test.py --auto
```

### Custom Serial Port
```bash
python3 ultrasonic_test.py -p /dev/ttyACM1
```

### Custom Baud Rate
```bash
python3 ultrasonic_test.py -b 9600
```

## Calibration

Before running scans, calibrate the rotation timing:

1. Run: `python3 ultrasonic_test.py`
2. Enter command: `c`
3. Robot rotates for 1 second
4. Measure the angle rotated
5. Calculate: `ROTATION_TIME_PER_10DEG = 1000 * 10 / measured_angle`
6. Update the value in `ultrasonic_test.ino` line 25
7. Re-upload Arduino sketch

## Output

The scan will:
- Rotate 360° taking measurements every 10° (36 total)
- Display each measurement in real-time
- Report the furthest wall angle and distance
- Report the closest wall angle and distance

## Example Output

```
Starting 360° Ultrasonic Scan
==================================================

Angle 0°: 145 cm
Angle 10°: 152 cm
Angle 20°: 158 cm
...

==================================================
Scan Results Summary
==================================================

Furthest wall detected at:
  Angle: 90°
  Distance: 245 cm

Closest wall detected at:
  Angle: 270°
  Distance: 98 cm
```

## Troubleshooting

**Can't connect to serial port:**
- Check port with `ls /dev/tty*`
- Add user to dialout group: `sudo usermod -a -G dialout $USER`
- Logout and login again

**No scan data:**
- Check Arduino is running (LED should be on)
- Verify baud rate matches (115200)
- Try unplugging/replugging USB

**Rotation is inaccurate:**
- Run calibration mode (`c` command)
- Adjust `ROTATION_TIME_PER_10DEG` in Arduino code
- May need different values for different battery levels
