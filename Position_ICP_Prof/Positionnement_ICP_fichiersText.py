# -*- coding: utf-8 -*-
"""
Created on Fri Dec 12 18:32:37 2025

@author: fabrice.roy
"""


# localisation relative avec ICP (Open3D)

import open3d as o3d
import numpy as np


# Charge deux scans consecutifs LiDAR
#Rotation : ~5° autour de l’axe Z
#Translation : ≈ (+0.05, +0.02, 0)


# Conversion TXT -> PCD

points = np.loadtxt("scan_1.txt")  # shape (N, 3)

# Créer le point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Sauvegarder en PCD
o3d.io.write_point_cloud("scan_1.pcd", pcd, write_ascii=True)

points = np.loadtxt("scan_2.txt")  # shape (N, 3)

# Créer le point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Sauvegarder en PCD
o3d.io.write_point_cloud("scan_2.pcd", pcd, write_ascii=True)




scan1 = o3d.io.read_point_cloud("scan_1.pcd")
scan2 = o3d.io.read_point_cloud("scan_2.pcd")



# Voxelisation pour accélérer l'ICP
voxel_size = 0.2
scan1_ds = scan1.voxel_down_sample(voxel_size)
scan2_ds = scan2.voxel_down_sample(voxel_size)



# ICP
threshold = 1.0  # distance max pour les correspondances
trans_init = np.eye(4)


reg = o3d.pipelines.registration.registration_icp(
    scan2_ds, scan1_ds, threshold, trans_init,
    #o3d.pipelines.registration.TransformationEstimationPointToPlane()
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

print("Transformation estimée :\n", reg.transformation)

'''
# réponse attendue
Rz(3°) =
[ 0.998  -0.052   0 ]
[ 0.052   0.998   0 ]
[ 0       0       1 ]

t = [-0.07, -0.07, 0]   10 cm
'''


'''

# ---------------------------------------------------------

map_pcd = o3d.io.read_point_cloud("map.pcd")
scan = o3d.io.read_point_cloud("current_scan.pcd")

voxel = 0.5
map_ds = map_pcd.voxel_down_sample(voxel)
scan_ds = scan.voxel_down_sample(voxel)

init_guess = np.eye(4)

result = o3d.pipelines.registration.registration_ndt(
    scan_ds, map_ds, 1.0, init_guess
)

print("Pose sur la carte :\n", result.transformation
'''