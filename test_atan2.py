import math

x_from = 0.5
y_from = 0.1
x_to = 0.2
y_to = 0.1

dX = x_to - x_from
dY = y_to - y_from

heading_angle = math.atan2(dX, dY) * 180 / math.pi
heading_angle = (heading_angle + 360) % 360

print(f"heading_angle: {heading_angle}")