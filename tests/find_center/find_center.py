import math

values = [
    3,
    3.067,
    3.2839,
    3.7082,
    3.3641,
    2.8867,
    2.6287,
    2.5138,
    2.5138,
    2.6287,
    2.8868,
    3.3641,
    4.2533,
    6.1465,
    9.201,
    9,
    9.201,
    9.8517,
    11.058,
    8.7460,
    7.5056,
    6.8345,
    6.536,
    6.536,
    6.8345,
    6,
    4.4834,
    3.7082,
    3.2839,
    3.067,
    3
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