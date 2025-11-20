#!/usr/bin/env python3
"""
IR Scanning Test with Panning Motion
- Pans back and forth within ~90° range
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
PAN_DURATION = 3.0       # Seconds to pan in each direction (~90° total range)
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
            # Require ≥20% signal to stop (≥4 out of 20 samples)
            if zeros >= self.samples * 0.2:
                self.state = 0
        else:  # Currently DETECTED (Stopped)
            # Require <10% signal to start moving again (<2 out of 20 samples)
            if zeros < self.samples * 0.1:
                self.state = 1
                
        return self.state


class PanningScanner:
    def __init__(self):
        self.direction = 1  # 1 = right, -1 = left
        self.pan_start_time = time.time()
        self.stopped = False
        self.stop_timestamp = 0
        
    def get_motor_speeds(self, ir_detected):
        """
        Calculate motor speeds based on IR detection and panning state
        Returns (left_speed, right_speed) for differential drive rotation
        """
        current_time = time.time()
        
        # If beacon detected, stop and hold
        if ir_detected:
            if not self.stopped:
                print("BEACON DETECTED - STOPPING")
                self.stopped = True
                self.stop_timestamp = current_time
            return (0, 0)  # Stop motors
        
        # If we were stopped, check if minimum stop duration has passed
        if self.stopped:
            if current_time - self.stop_timestamp < STOP_DURATION:
                # Still in minimum stop period
                return (0, 0)
            else:
                # Resume panning
                print("RESUMING SCAN")
                self.stopped = False
                self.pan_start_time = current_time
        
        # Calculate panning motion
        time_in_direction = current_time - self.pan_start_time
        
        # Check if we need to reverse direction
        if time_in_direction >= PAN_DURATION:
            self.direction *= -1  # Reverse direction
            self.pan_start_time = current_time
            print(f"Reversing direction: {'RIGHT' if self.direction > 0 else 'LEFT'}")
        
        # Set motor speeds for rotation
        if self.direction > 0:
            # Pan right (clockwise)
            return (PAN_SPEED, PAN_SPEED)
        else:
            # Pan left (counter-clockwise)
            return (-PAN_SPEED, -PAN_SPEED)


async def main():
    print("=" * 50)
    print("IR PANNING SCANNER TEST")
    print("=" * 50)
    print(f"Pan range: ~90° (±45° from center)")
    print(f"Pan speed: {PAN_SPEED}")
    print(f"Direction switch: every {PAN_DURATION}s")
    print(f"Stop hold time: {STOP_DURATION}s")
    print("=" * 50)

    ser = serial.Serial('/dev/ttyACM0', 115200)
    ser.reset_input_buffer()

    ir_monitor = IRMonitor(SENSOR_1, samples=20)
    scanner = PanningScanner()

    try:
        while True:
            # Read IR sensor
            ir_state = ir_monitor.read_stable()
            ir_detected = (ir_state == 0)  # 0 = detected
            
            # Get motor speeds based on detection state
            left_speed, right_speed = scanner.get_motor_speeds(ir_detected)
            
            # Send motor commands to Arduino
            command = f"<{left_speed},{right_speed}>"
            ser.write(command.encode())
            
            # Status display
            if ir_detected and not scanner.stopped:
                print("IR DETECTED")
            elif scanner.stopped:
                elapsed = time.time() - scanner.stop_timestamp
                print(f"HOLDING ({elapsed:.1f}s / {STOP_DURATION}s)")
            
            await asyncio.sleep(0.1)  # 10 Hz update rate

    except KeyboardInterrupt:
        print("\n\nExiting...")
        ser.write(b"<0,0>")  # Stop motors
        GPIO.cleanup()
        ser.close()

if __name__ == "__main__":
    asyncio.run(main())
