import math

values = [
    1.022,
    1.094,
    1.236,
    1.494,
    2,
    2.628,
    2.513,
    2.513,
    2.628,
    2.886,
    3.364,
    4.253,
    6.146,
    11.245,
    11,
    11.245,
    12.041,
    11.058,
    8.746,
    7.505,
    6.834,
    6.535,
    6.535,
    3.236,
    2,
    1.494,
    2.236,
    1.094,
    1.022,
    1
]

x = 0.0
y = 0.0

EXPONENT = 1.8
NUM_DATA_POINTS = len(values)
step = 2*math.pi / NUM_DATA_POINTS

for i, d in enumerate(values):
    a = d * math.cos(i * step)
    if a < 0:
        a = -math.pow(-a, EXPONENT)
    else:
        a = math.pow(a, EXPONENT)

    b = d * math.sin(i * step)
    if b < 0:
        b = -math.pow(-b, EXPONENT)
    else:
        b = math.pow(b, EXPONENT)
    x += a
    y += b

x /= NUM_DATA_POINTS
y /= NUM_DATA_POINTS

x /= math.pow(abs(x), 1/EXPONENT)
y /= math.pow(abs(y), 1/EXPONENT)

print(x)
print(y)

print(math.atan2(y, x) / math.pi * 180) 