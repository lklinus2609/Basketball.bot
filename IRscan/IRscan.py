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
PAN_DURATION = 1.5       # Seconds to pan in each direction (~90° total range)
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
        
    def get_motor_command(self):
        """
        Calculate desired motor speed based on panning state
        Arduino will handle stopping when beacon detected
        """
        current_time = time.time()
        time_in_direction = current_time - self.pan_start_time
        
        # Check if we need to reverse direction
        if time_in_direction >= PAN_DURATION:
            self.direction *= -1  # Reverse direction
            self.pan_start_time = current_time
            print(f"Reversing direction: {'RIGHT' if self.direction > 0 else 'LEFT'}")
        
        # Return motor speed (Arduino handles stopping based on IR)
        if self.direction > 0:
            return PAN_SPEED  # Pan right
        else:
            return -PAN_SPEED  # Pan left


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
            
            # Get desired motor speed (panning direction)
            motor_speed = scanner.get_motor_command()
            
            # Send motor command + IR state to Arduino
            # Arduino will handle 500ms stop timer
            command = f"<{motor_speed},{ir_state}>"
            ser.write(command.encode())
            
            # Status display
            if ir_state == 0:
                print("IR DETECTED")
            
            await asyncio.sleep(0.1)  # 10 Hz update rate

    except KeyboardInterrupt:
        print("\n\nExiting...")
        ser.write(b"<0,1>")  # Stop motors
        GPIO.cleanup()
        ser.close()

if __name__ == "__main__":
    asyncio.run(main())
