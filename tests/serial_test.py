#!/usr/bin/env python
import time
import serial
import struct

ser = serial.Serial(
        port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)

time.sleep(3)
while True:
	#ser.write(bytearray([0x01, 60, 60, 200, 0]))
	ser.write(bytearray([0x05, 2]))
	start_time = time.time()
	#time.sleep(0.05)
	val = ser.read(2)
	receive_time = time.time()
	print(struct.unpack('h', val)[0])
	print((receive_time - start_time) * 1000000)