# simple linefollowing program for RasPi
# TODO: i2c connection to Teensy

import cv2
from smbus import SMBus

cap = cv2.VideoCapture('test_vid.mp4')
FRAME_HEIGHT = 204
FRAME_WIDTH = 364

i2cbus = SMBus(1)

def _map(x, in_min, in_max, out_min, out_max):
	return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

while True:
	_, frame = cap.read()
	frame = cv2.inRange(frame, (0, 0, 0), (255, 255, 75))
	print(frame.shape[0], frame.shape[1])
	
	frame_left = frame[0:FRAME_HEIGHT, 0:int(FRAME_WIDTH/2)]
	frame_right = frame[0:FRAME_HEIGHT, int(FRAME_WIDTH/2):FRAME_WIDTH]

	black_px_left = cv2.countNonZero(frame_left)
	black_px_right = cv2.countNonZero(frame_right)

	delta = black_px_left - black_px_right
	motor_speed = _map(delta, -FRAME_HEIGHT*FRAME_WIDTH, FRAME_HEIGHT*FRAME_WIDTH, -255, 255)
	print(motor_speed)
	cv2.imshow("original", frame)
	cv2.imshow("left", frame_left)
	cv2.imshow("right", frame_right)

	# send data to Teensy:
	i2cbus.write_byte(0x2a, motor_speed)
	print("Data sent")
	cv2.waitKey(0)

