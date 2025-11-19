#!/usr/bin/env python3
import serial
import time
import numpy as np
import RPi.GPIO as GPIO
import asyncio

sensor = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor,GPIO.IN)

sensor2 = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor2,GPIO.IN)


async def IRcheck(ser):
	await asyncio.sleep(1)
	irValue = GPIO.input(sensor)
	irValue2 = GPIO.input(sensor2)
	serial.Serial('/dev/ttyACM0',115200).write(('<'+str(irValue)+','+str(irValue2)+'>').encode())
	
	if irValue:
		print('ir1 NOT DETECTED')
				
	else:
		print('ir1 DETECTED')
	
	if irValue2:
		print('ir2 NOT DETECTED')
				
	else:
		print('ir2 DETECTED')

async def main():
	print('IR Sensor Ready')
	ser = serial.Serial('/dev/ttyACM0',115200)
	ser.reset_input_buffer()
	try:
		while True:
			await IRcheck(ser)
	except KeyboardInterrupt:
		GPIO.cleanup()

asyncio.run(main())
