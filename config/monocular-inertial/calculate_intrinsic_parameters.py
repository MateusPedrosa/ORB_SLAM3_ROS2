# Replace the values with the values from Isaac Sim

import math

width = 1920
height = 1080
aspect_ratio = width / height

focal_length =21.0
horiz_aperture = 20.955
# Pixels are square so we can do:
vert_aperture = 11.78719
fov = 2 * math.atan(horiz_aperture / (2 * focal_length))

# compute focal point and center
focal_x = width * focal_length / horiz_aperture
focal_y = height * focal_length / vert_aperture
center_x = width * 0.5
center_y = height * 0.5

print('focal_x:', focal_x)
print('focal_y:', focal_y)
print('center_x:', center_x)
print('center_y', center_y)