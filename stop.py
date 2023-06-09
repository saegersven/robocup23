#!/usr/bin/env python
import time
import serial
import struct

try:
        ser = serial.Serial(
                port='/dev/ttyUSB0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
                baudrate = 115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
        )

        time.sleep(2)
        ser.write(bytearray([0x02]))
        ser.close()

        print("--- Stopped motors from python script ---")

except:
        print("Coudn't stop motors, no NANO on USB0.")