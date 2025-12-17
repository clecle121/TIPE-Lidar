# -*- coding: utf-8 -*-
"""
solution avec carte existante carte.txt

# Nota : Bien fermer les fenetres graphiques !!
# Faire : lidar.disconnect() si mauvaise sortie précédente
"""

import matplotlib.pyplot as plt
import numpy as np
import math
import time
from rplidar import RPLidar
import os
import open3d as o3d

# ***************************************************************
# Paramètres
PORT = 'COM4'
lidar = RPLidar(PORT, baudrate=115200)
lidar.start_motor()
time.sleep(1)
scan_gen = lidar.iter_scans(max_buf_meas=4000)

save_dir = "localized_scans_txt"
os.makedirs(save_dir, exist_ok=True)

# ***************************************************************
# Charger la carte existante
# La carte doit être un nuage de points Nx3
# Format PCD ou TXT
# Exemple : carte_txt = np.loadtxt("map_existing.txt")
# Ici pour test, on crée un nuage vide (remplacer par vraie carte)
global_map = np.loadtxt("carte.txt")  # Nx3 array

pc_map = o3d.geometry.PointCloud()
pc_map.points = o3d.utility.Vector3dVector(global_map)
pc_map = pc_map.voxel_down_sample(0.05)
global_map = np.asarray(pc_map.points)

# ***************************************************************
def get_scan_points():
    """Récupère un scan LiDAR en coordonnées (x, y, 0) mètres."""
    scan = next(scan_gen)
    pts = []
    for (_, angle, distance) in scan:
        if 0.1 < distance/1000.0 < 6.0:
            d = distance / 1000.0
            angle_rad = math.radians(angle)
            x = d * math.cos(angle_rad)
            y = d * math.sin(angle_rad)
            pts.append([x, y, 0.0])
    return np.asarray(pts)

# ***************************************************************
def icp_registration(source_pts, target_pts, init=np.eye(4)):
    """ICP point-to-point pour localisation."""
    source = o3d.geometry.PointCloud()
    target = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(source_pts)
    target.points = o3d.utility.Vector3dVector(target_pts)

    source = source.voxel_down_sample(0.05)
    target = target.voxel_down_sample(0.05)

    threshold = 0.2
    reg = o3d.pipelines.registration.registration_icp(
        source, target, threshold, init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return reg.transformation

# ***************************************************************
# Initialisation
prev_scan = get_scan_points()
pose = np.eye(4)  # Pose initiale sur la carte
trajectory = []

scan_skip = 0
SKIP_N = 3
scan_index = 0

# Matplotlib 2D
plt.ion()
fig, ax = plt.subplots(figsize=(6,6))
scatter = ax.scatter(prev_scan[:,0], prev_scan[:,1], s=2, c='red', label='scan courant')
traj_line, = ax.plot([], [], 'g-', lw=1, label='trajectoire')
ax.scatter(global_map[:,0], global_map[:,1], s=1, c='blue', label='carte existante')
ax.set_aspect('equal')
ax.set_xlim(-6,6)
ax.set_ylim(-6,6)
ax.set_title('Localisation LiDAR sur carte existante')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.grid(True)
ax.legend()

# ***************************************************************
try:
    while True:
        scan_skip += 1
        curr_scan = get_scan_points()
        if scan_skip % SKIP_N != 0 or len(curr_scan) == 0:
            continue

        # ----- ICP point-to-point sur carte existante -----
        T = icp_registration(curr_scan, global_map, init=pose)
        pose = T

        homog = np.hstack((curr_scan, np.ones((len(curr_scan),1))))
        curr_scan_tf = (pose @ homog.T).T[:, :3]

        # ----- Mise à jour trajectoire -----
        trajectory.append([pose[0,3], pose[1,3]])

        # ----- Affichage Matplotlib -----
        scatter.set_offsets(curr_scan_tf[:,:2])
        traj_line.set_data(np.array(trajectory)[:,0], np.array(trajectory)[:,1])
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        # ----- Sauvegarde scan transformé -----
        filename = os.path.join(save_dir, f"scan_{scan_index:05d}.txt")
        np.savetxt(filename, curr_scan_tf, fmt="%.6f", delimiter=" ", header="x y z", comments="")
        scan_index += 1

        prev_scan = curr_scan
        time.sleep(0.02)

except KeyboardInterrupt:
    print("Arrêt utilisateur")

finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
