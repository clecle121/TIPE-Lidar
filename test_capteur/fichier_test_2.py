'''from rplidar import RPLidar

lidar = RPLidar('COM5', baudrate=115200)

try:
    for scan, quality, angle, distance in lidar.iter_measurments():
        pass
except(RuntimeError, TypeError, NameError):
    print('erreur')
finally:
    print('finally')'''

import numpy as np
import os

os.chdir('position_ICP_Prof')

global_map = np.loadtxt("carte.txt", skiprows=1)

print(global_map)