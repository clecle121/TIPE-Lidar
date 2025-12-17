# -*- coding: utf-8 -*-
"""
Solution avec carte cumulée (limite la dérive) et loop closure (reconnaissance point )

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
from scipy.spatial import cKDTree

# ***************************************************************
PORT = 'COM4'
lidar = RPLidar(PORT, baudrate=115200)
lidar.start_motor()
time.sleep(1)
scan_gen = lidar.iter_scans(max_buf_meas=6000)     # 4000- 6000 max : diminue la dérive

save_dir = "lidar_scans_txt"
os.makedirs(save_dir, exist_ok=True)

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
def icp_registration(source_pts, target_pts):
    """ICP point-to-point sur Open3D."""
    source = o3d.geometry.PointCloud()
    target = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(source_pts)
    target.points = o3d.utility.Vector3dVector(target_pts)

    # Downsample pour accélérer ICP et réduire bruit
    source = source.voxel_down_sample(0.05)
    target = target.voxel_down_sample(0.05)

    threshold = 0.2  # mètres
    init = np.eye(4)

    reg = o3d.pipelines.registration.registration_icp(
        source, target, threshold, init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return reg.transformation

# ***************************************************************
# Initialisation
prev_scan = get_scan_points()
pose = np.eye(4)
trajectory = []

scan_skip = 0
SKIP_N = 6                          # impacte la dérive
scan_index = 0

# Carte cumulée pour ICP global
global_map = prev_scan.copy()

# Matplotlib 2D
plt.ion()
fig, ax = plt.subplots(figsize=(6,6))
scatter = ax.scatter(prev_scan[:,0], prev_scan[:,1], s=2, c='blue')
traj_line, = ax.plot([], [], 'g-', lw=1)
ax.set_aspect('equal')
ax.set_xlim(-6, 6)
ax.set_ylim(-6, 6)
ax.set_title('RPLidar 2D – ICP Point-to-Point Anti-Dérive')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.grid(True)

# ***************************************************************
try:
    while True:
        scan_skip += 1
        curr_scan = get_scan_points()
        if scan_skip % SKIP_N != 0 or len(curr_scan) == 0:
            continue

        # ----- ICP point-to-point sur carte cumulée -----
        T = icp_registration(curr_scan, global_map)
        pose = pose @ T

        homog = np.hstack((curr_scan, np.ones((len(curr_scan),1))))
        curr_scan_tf = (pose @ homog.T).T[:, :3]

        # ----- Mise à jour carte cumulée (downsample) -----
        global_map = np.vstack((global_map, curr_scan_tf))
        pc_map = o3d.geometry.PointCloud()
        pc_map.points = o3d.utility.Vector3dVector(global_map)
        pc_map = pc_map.voxel_down_sample(0.05)
        global_map = np.asarray(pc_map.points)

        # ----- Loop closure simple : translation uniquement -----
        if len(trajectory) > 50:
            tree = cKDTree(global_map[:,:2])
            dist, idx = tree.query(curr_scan_tf[:,:2], k=1)
            close_points = dist < 0.5
            if np.any(close_points):
                mean_target = np.mean(global_map[idx[close_points]], axis=0)
                mean_scan = np.mean(curr_scan_tf[close_points], axis=0)
                delta = mean_target - mean_scan
                pose[0:3,3] += delta

        # ----- Trajectoire -----
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
