'''
from rplidar import RPLidar
from numpy import *

lidar = RPLidar('COM5')

for new_scan, quality, angle, distance in lidar.iter_measurments():
    #print(f"Angle: {angle:.2f}°  |  Distance: {distance:.2f} mm  |  Qualité: {quality}")
    angle = deg2rad(angle)
    print([cos(angle), -sin(angle)], "\n", [sin(angle), cos(angle)], "\n", "\n")

try:
    for new_scan, quality, angle, distance in lidar.iter_measurments():
        print(f"Angle: {angle:.2f}°  |  Distance: {distance:.2f} mm  |  Qualité: {quality}")

except KeyboardInterrupt:
    print("Arrêt du scan")

finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
'''