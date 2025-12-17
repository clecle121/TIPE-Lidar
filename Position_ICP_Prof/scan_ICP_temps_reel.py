# -*- coding: utf-8 -*-
"""
Created on Tue Dec 16 15:33:23 2025
Améliorations possibles :
    downsampling (voxel_down_sample)
    filtrage distance max
    point-to-plane ICP
    rejection des outliers
"""

import matplotlib.pyplot as plt
import numpy as np
import math
import time
from rplidar import RPLidar

# ***************************************************************
PORT = 'COM4'
lidar = RPLidar(PORT, baudrate=115200)

lidar.start_motor()
time.sleep(1)

scan_gen = lidar.iter_scans(max_buf_meas=4000)

# ***************************************************************
def get_scan_points():
    scan = next(scan_gen)
    pts = []

    for (_, angle, distance) in scan:
        if 0.1 < distance/1000.0 < 6.0:  # filtrage 10 cm à 6 m
            d = distance / 1000.0         # mm → m
            angle_rad = math.radians(angle)
            x = d * math.cos(angle_rad)
            y = d * math.sin(angle_rad)
            pts.append([x, y])

    return np.asarray(pts)

# ***************************************************************
# Initialisation matplotlib
plt.ion()
fig, ax = plt.subplots(figsize=(6,6))

scatter = ax.scatter([], [], s=2, c='blue')
traj_line, = ax.plot([], [], 'g-', lw=1)  # trajectoire

ax.set_aspect('equal')
ax.set_xlim(-6, 6)
ax.set_ylim(-6, 6)
ax.set_title('RPLidar 2D – Temps réel')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.grid(True)

# ***************************************************************
# Variables de trajectoire et buffer
trajectory = []
scan_skip = 0
SKIP_N = 3  # traiter 1 scan sur 3 pour éviter overflow

# ***************************************************************
try:
    while True:
        scan_skip += 1
        pts = get_scan_points()

        if scan_skip % SKIP_N != 0 or len(pts) == 0:
            continue  # on jette ce scan

        # Mise à jour des points du Lidar
        scatter.set_offsets(pts)

        # Mise à jour de la trajectoire (position du centre du LiDAR)
        # Ici on suppose que le LiDAR reste au centre (0,0)
        # Tu peux remplacer par ICP pour suivre le mouvement
        trajectory.append([0, 0])  
        traj_line.set_data(np.array(trajectory)[:,0], np.array(trajectory)[:,1])

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        time.sleep(0.02)  # ~50 FPS

except KeyboardInterrupt:
    print("Arrêt utilisateur")

finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
