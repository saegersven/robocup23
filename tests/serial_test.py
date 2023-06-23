#!/usr/bin/env python
import time
import serial
import struct

ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)

time.sleep(3)
while True:
	#ser.write(bytearray([0x01, 60, 60, 200, 0]))
	ser.write(bytearray([0x05, 0]))
	val = ser.read(2)
	print(struct.unpack('<h', val)[0])