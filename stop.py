# stops motors during compilation
import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(9, GPIO.OUT)
print("--- Stopped motors from python script ---")
GPIO.output(9, GPIO.HIGH)
time.sleep(0.01)
GPIO.output(9, GPIO.LOW)
time.sleep(0.1)
GPIO.cleanup()