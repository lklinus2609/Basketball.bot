#!/usr/bin/env python3
"""
IR Scanning Test with Panning Motion
- Pans back and forth within ~90 degree range
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
    Simple scanner - sends constant speed to Arduino
    Arduino handles all panning direction control via encoders
    """
    def get_motor_command(self):
        """Return constant motor speed - Arduino controls direction"""
        return PAN_SPEED  # Always send positive speed, Arduino handles reversing


async def main():
    print("=" * 50)
    print("IR PANNING SCANNER TEST - DEBUG MODE")
    print("=" * 50)
    print(f"Pan range: ~90 deg (+/- 45 deg from center)")
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
    print("DEBUG: Loop | IR_State | Samples(0s/20) | Cmd | Notes")
    print("-" * 70)
    
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
            
            # Get motor command
            motor_speed = scanner.get_motor_command()
            
            # Send command to Arduino
            command = f"<{motor_speed},{ir_state}>"
            ser.write(command.encode())
            
            # READ SERIAL to prevent Arduino blocking!
            # Arduino sends debug info every 500ms. If we don't read it, 
            # Arduino's TX buffer fills up and Serial.print() BLOCKS.
            arduino_debug = ""
            while ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        arduino_debug = line  # Keep last line for display
                except Exception:
                    pass
            
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
            
            # Print every loop for detailed tracking
            loop_count += 1
            state_str = "DETECT" if ir_state == 0 else "CLEAR "
            
            # Append Arduino debug info if available
            debug_str = f" | Ard: {arduino_debug}" if arduino_debug else ""
            
            print(f"{loop_count:5d} | {state_str} | {zeros:2d}/20 ({zeros*5:3d}%) | {command:10s} | {' '.join(notes)}{debug_str}")
            
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

