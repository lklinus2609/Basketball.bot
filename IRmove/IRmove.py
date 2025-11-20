#!/usr/bin/env python3
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

class IRMonitor:
    def __init__(self, pin, samples=5):
        self.pin = pin
        self.samples = samples
        self.state = 1  # 1 = beam unbroken, 0 = beam blocked

    def read_stable(self):
        vals = [GPIO.input(self.pin) for _ in range(self.samples)]
        zeros = vals.count(0)  # Count "detected" samples (LOW)

        # Hysteresis Logic (Sticky State)
        # Prevents "chatter" when at the edge of the beam
        
        if self.state == 1:  # Currently NOT DETECTED (Moving)
            # Require ≥20% signal to stop (≥4 out of 20 samples)
            if zeros >= self.samples * 0.2:
                self.state = 0
        else:  # Currently DETECTED (Stopped)
            # Require <10% signal to start moving again (<2 out of 20 samples)
            if zeros < self.samples * 0.1:
                self.state = 1
                
        return self.state


async def IRcheck(ir_monitor, ser):
    ir_state = ir_monitor.read_stable()

    ser.write(f"<{ir_state}>".encode())

    if ir_state:
        print("IR beam NOT DETECTED")
    else:
        print("IR beam DETECTED")

    await asyncio.sleep(0.1)  # 10 ms loop for faster response


async def main():
    print("IR Sensor Ready")

    ser = serial.Serial('/dev/ttyACM0', 115200)
    ser.reset_input_buffer()

    ir_monitor = IRMonitor(SENSOR_1, samples=20)

    try:
        while True:
            await IRcheck(ir_monitor, ser)
    except KeyboardInterrupt:
        print("Exiting...")
        GPIO.cleanup()
        ser.close()

if __name__ == "__main__":
    asyncio.run(main())
