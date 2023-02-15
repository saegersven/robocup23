import numpy as np
import math
from matplotlib import pyplot as plt

# Zone definition
w = 120
h = 90

# Center of zone
C = np.array((w, h)) / 2

err = []

N = 1000
for i in range(N):
	# Robot position
	P = np.array((np.random.uniform() * w, np.random.uniform() * h))

	# Vector from robot to center (what we want to compute without knowing the robots position)
	Q = C - P

	# List of distance vectors (readings from distance sensor)
	D = []

	# Number of measurements
	n = 50

	# Distance function depending on robot position (p) and angle relative to x axis (right, going ccw)
	def d(p, alpha):
		if alpha < 0:
			alpha = alpha + math.pi * 2
		elif alpha > math.pi * 2:
			alpha = alpha - math.pi * 2
			
		if alpha > 0:
			if alpha < math.pi / 2:
				right_wall_upper_angle = math.atan((h - P[1]) / (w - P[0]))
				
				if alpha < right_wall_upper_angle:
					# Right wall
					return (w - P[0]) / math.cos(alpha)

				# Upper wall
				return (h - P[1]) / math.cos(math.pi / 2 - alpha)
			elif alpha < math.pi:
				upper_wall_upper_angle = math.atan(P[0] / (h - P[1])) + math.pi / 2

				if alpha < upper_wall_upper_angle:
					# Upper wall
					return (h - P[1]) / math.cos(alpha - math.pi / 2)

				# Left wall
				return P[0] / math.cos(math.pi - alpha)
			elif alpha < math.pi * 3 / 2:
				left_wall_upper_angle = math.atan(P[1] / P[0]) + math.pi

				if alpha < left_wall_upper_angle:
					# Left wall
					return P[0] / math.cos(alpha - math.pi)

				# Lower wall
				return P[1] / math.cos(math.pi * 3 / 2 - alpha)
			else:
				lower_wall_upper_angle = math.atan((w - P[0]) / P[1]) + math.pi * 3 / 2

				if alpha < lower_wall_upper_angle:
					# Lower wall
					return P[1] / math.cos(alpha - math.pi * 3 / 2)

				# Right wall
				return (w - P[0]) / math.cos(math.pi * 2 - alpha)

	x = []
	y = []
	sx = 0
	sy = 0
	for i in range(n):
		alpha = math.pi * 2 / n * i

		d_ = d(P, alpha)
		if d_ is not None:

			sx += d_**2 * math.cos(alpha)
			sy += d_**2 * math.sin(alpha)

	#plt.plot(x, y)
	#plt.show()

	sx /= n
	sy /= n
	s = np.array((sx, sy))
	l = np.linalg.norm(s)
	s /= math.sqrt(l)

	err.append(np.linalg.norm(s - Q))

print(np.mean(err))
print(np.max(err))