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
        self.state = 0 if sum(vals) < self.samples / 2 else 1
        return self.state


async def IRcheck(ir_monitor, ser):
    ir_state = ir_monitor.read_stable()

    ser.write(f"<{ir_state}>".encode())

    if ir_state:
        print("IR beam NOT DETECTED")
    else:
        print("IR beam DETECTED")

    await asyncio.sleep(0.05)  # 100 ms loop for fast response


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
