#!/usr/bin/env python3
"""
IR Scanning with Ultrasonic Wall Detection
- Uses ultrasonic sensor to detect walls and reverse scan direction
- Adapts to robot position - no hard-coded rotation limits
- Stops when IR beacon detected
- Holds for 0.5s minimum before resuming
"""
import serial
import time
import RPi.GPIO as GPIO
import asyncio

# -------------------------------
# GPIO Setup
# -------------------------------
SENSOR_1 = 17  

GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_1, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# -------------------------------
# Panning Configuration
# -------------------------------
PAN_SPEED = 75           # Motor speed for panning
PAN_DURATION = 1.5       # Seconds to pan in each direction (~90 deg total range)
STOP_DURATION = 0.5      # Minimum stop time when beacon detected
WALL_THRESHOLD = 100    # Distance in cm - turn when closer than this to wall
DIRECTION_CHANGE_COOLDOWN = 0.5 # Seconds to wait after changing direction before allowing another change

class IRMonitor:
    def __init__(self, pin, samples=20):
        self.pin = pin
        self.samples = samples
        self.state = 1  # 1 = beam unbroken, 0 = beam blocked

    def read_stable(self):
        vals = [GPIO.input(self.pin) for _ in range(self.samples)]
        zeros = vals.count(0)  # Count "detected" samples (LOW)

        # Hysteresis Logic (20% detect, 10% clear)
        if self.state == 1:  # Currently NOT DETECTED (Moving)
            # Require >=20% signal to stop (>=4 out of 20 samples)
            if zeros >= self.samples * 0.2:
                self.state = 0
        else:  # Currently DETECTED (Stopped)
            # Require <10% signal to start moving again (<2 out of 20 samples)
            if zeros < self.samples * 0.1:
                self.state = 1
                
        return self.state


class PanningScanner:
    """
    Scanner with ultrasonic-based wall detection
    Sends turn_flag to Arduino based on wall proximity
    """
    def __init__(self):
        self.current_direction = -1  # -1 = left, 1 = right
        self.ultrasonic_distance = 999  # Current distance reading
        self.last_direction_change_time = 0  # Track when we last changed direction
        self.has_reached_threshold = False  # Track if we've reached the safe distance threshold
        self.distance_buffer = []  # Store recent distance readings for filtering
        self.buffer_size = 5  # Number of readings to keep
    
    def update_distance(self, distance):
        """Update current ultrasonic distance with noise filtering"""
        # Add to buffer
        self.distance_buffer.append(distance)
        if len(self.distance_buffer) > self.buffer_size:
            self.distance_buffer.pop(0)
        
        # Use median of buffer to filter out noise spikes
        if len(self.distance_buffer) >= 3:
            sorted_distances = sorted(self.distance_buffer)
            filtered_distance = sorted_distances[len(sorted_distances) // 2]
        else:
            filtered_distance = distance
        
        self.ultrasonic_distance = filtered_distance
        
        # Check if we've reached the threshold distance
        # Ignore readings > 500cm as likely errors
        if filtered_distance >= WALL_THRESHOLD and filtered_distance < 500:
            self.has_reached_threshold = True
    
    def get_turn_flag(self):
        """
        Determine turn direction based on wall proximity
        Returns: 0 = normal, 1 = turn right, -1 = turn left
        
        Only allows direction change if:
        1. Currently close to wall (distance < threshold)
        2. Cooldown period has elapsed since last change
        3. Robot has reached the safe distance threshold at least once
        """
        import time
        turn_flag = 0
        current_time = time.time()
        
        # Check if enough time has passed since last direction change
        time_since_change = current_time - self.last_direction_change_time
        
        # Only turn if:
        # - Currently close to wall (< WALL_THRESHOLD)
        # - Cooldown has expired
        # - We have reached the threshold distance at least once
        if (self.ultrasonic_distance < WALL_THRESHOLD and 
            time_since_change >= DIRECTION_CHANGE_COOLDOWN and
            self.has_reached_threshold):
            
            # Too close to wall, reverse direction
            if self.current_direction == -1:
                # Was going left, now go right
                turn_flag = 1
                self.current_direction = 1
                self.last_direction_change_time = current_time
                self.has_reached_threshold = False  # Reset flag
            elif self.current_direction == 1:
                # Was going right, now go left
                turn_flag = -1
                self.current_direction = -1
                self.last_direction_change_time = current_time
                self.has_reached_threshold = False  # Reset flag
        
        return turn_flag
    
    def get_motor_command(self):
        """Return constant motor speed - Arduino controls direction"""
        return PAN_SPEED  # Always send positive speed, Arduino handles reversing


async def main():
    print("=" * 50)
    print("IR SCANNER WITH ULTRASONIC WALL DETECTION")
    print("=" * 50)
    print(f"Wall threshold: {WALL_THRESHOLD}cm")
    print(f"Pan speed: {PAN_SPEED}")
    print(f"Stop hold time: {STOP_DURATION}s")
    print("=" * 50)

    print("Opening serial connection...")
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        print("Serial connection opened!")
        ser.reset_input_buffer()
        print("Input buffer cleared")
    except Exception as e:
        print(f"ERROR opening serial port: {e}")
        print("Make sure Arduino is connected and code is uploaded")
        GPIO.cleanup()
        return

    print("Initializing monitors...")
    ir_monitor = IRMonitor(SENSOR_1, samples=20)
    scanner = PanningScanner()
    print("Ready to start panning!")
    print("=" * 50)
    print("DEBUG: Loop | IR_State | Samples(0s/20) | Dist(cm) | TurnFlag | Notes")
    print("-" * 80)
    
    loop_count = 0
    last_ir_state = ir_monitor.state
    last_command = ""

    try:
        while True:
            loop_start = time.time()
            
            # Read IR sensor with sample tracking
            vals = [GPIO.input(SENSOR_1) for _ in range(20)]
            zeros = vals.count(0)
            prev_state = ir_monitor.state
            
            # Manual hysteresis logic with tracking
            if ir_monitor.state == 1:  # Currently NOT DETECTED
                if zeros >= 4:  # 20% threshold
                    ir_monitor.state = 0
            else:  # Currently DETECTED
                if zeros < 2:  # 10% threshold
                    ir_monitor.state = 1
            
            ir_state = ir_monitor.state
            
            # READ SERIAL from Arduino to get ultrasonic distance
            arduino_debug = ""
            ultrasonic_dist = scanner.ultrasonic_distance  # Keep previous value as default
            
            while ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # Parse distance data (format: "D:123")
                        if line.startswith("D:"):
                            try:
                                dist = int(line[2:])
                                scanner.update_distance(dist)
                                ultrasonic_dist = dist
                            except ValueError:
                                pass
                        else:
                            arduino_debug = line  # Keep non-distance lines for debug
                except Exception:
                    pass
            
            # Get motor command and turn flag
            motor_speed = scanner.get_motor_command()
            turn_flag = scanner.get_turn_flag()
            
            # Send command to Arduino: <motor_speed,ir_state,turn_flag>
            command = f"<{motor_speed},{ir_state},{turn_flag}>"
            ser.write(command.encode())
            
            # Detect state changes
            state_changed = (ir_state != prev_state)
            command_changed = (command != last_command)
            
            # Print debug info every loop OR when state changes
            notes = []
            if state_changed:
                if ir_state == 0:
                    notes.append(">>> IR DETECTED! STOPPING")
                else:
                    notes.append(">>> IR CLEARED! RESUMING")
            
            if turn_flag != 0:
                if turn_flag == 1:
                    notes.append(f"WALL LEFT - TURN RIGHT (reached_threshold:{scanner.has_reached_threshold})")
                elif turn_flag == -1:
                    notes.append(f"WALL RIGHT - TURN LEFT (reached_threshold:{scanner.has_reached_threshold})")
            
            # Print every loop for detailed tracking
            loop_count += 1
            state_str = "DETECT" if ir_state == 0 else "CLEAR "
            turn_str = f"{turn_flag:+2d}"
            
            # Show threshold reached status for debugging
            threshold_str = f" Threshold:{'YES' if scanner.has_reached_threshold else 'NO '}"
            
            # Append Arduino debug info if available
            debug_str = f" | Ard: {arduino_debug}" if arduino_debug else ""
            
            print(f"{loop_count:5d} | {state_str} | {zeros:2d}/20 ({zeros*5:3d}%) | {ultrasonic_dist:3d}cm{threshold_str} | {turn_str} | {' '.join(notes)}{debug_str}")
            
            last_ir_state = ir_state
            last_command = command
            
            # Sleep to maintain 10Hz
            elapsed = time.time() - loop_start
            sleep_time = max(0, 0.1 - elapsed)
            await asyncio.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nExiting...")
        ser.write(b"<0,1>")  # Stop motors
        GPIO.cleanup()
        ser.close()


if __name__ == "__main__":
    asyncio.run(main())

