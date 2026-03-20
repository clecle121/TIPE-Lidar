# -*- coding: utf-8 -*-
"""
Solution avec carte cumulée (limite la dérive) et loop closure (reconnaissance de points)

Ce script utilise un LiDAR RPLidar pour cartographier un environnement en 2D.
Il implémente une méthode de SLAM (Simultaneous Localization and Mapping) simplifiée :
- ICP (Iterative Closest Point) pour corriger la dérive de l'odométrie.
- Loop closure pour détecter les boucles et corriger la position.
- Une carte cumulée pour améliorer la précision globale.

Nota :
- Bien fermer les fenêtres graphiques après utilisation pour libérer les ressources.
- Si une sortie précédente a été interrompue, exécuter lidar.disconnect() avant de relancer.
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
# Configuration du LiDAR
# PORT : Port série auquel le LiDAR est connecté (à adapter selon votre système)
# baudrate : Vitesse de communication en bauds
# max_buf_meas : Taille du buffer pour les mesures (plus grand = moins de dérive mais plus de latence)
PORT = 'COM4'
lidar = RPLidar(PORT, baudrate=115200)
lidar.start_motor()  # Démarre le moteur du LiDAR
time.sleep(1)  # Attend que le LiDAR soit prêt
scan_gen = lidar.iter_scans(max_buf_meas=6000)  # Générateur de scans LiDAR

# Répertoire pour sauvegarder les scans
save_dir = "lidar_scans_txt"
os.makedirs(save_dir, exist_ok=True)  # Crée le répertoire s'il n'existe pas

# ***************************************************************
def get_scan_points():
    """
    Récupère un scan LiDAR et le convertit en coordonnées cartésiennes (x, y, 0) en mètres.

    Retourne :
    - Un tableau numpy de points 3D (z=0) en mètres.
    - Filtre les points trop proches ou trop éloignés pour éviter le bruit.
    """
    scan = next(scan_gen)  # Récupère le prochain scan disponible
    pts = []
    for (_, angle, distance) in scan:
        # Filtre les distances entre 0.1m et 6m
        if 0.1 < distance/1000.0 < 6.0:
            d = distance / 1000.0  # Convertit en mètres
            angle_rad = math.radians(angle)  # Convertit l'angle en radians
            x = d * math.cos(angle_rad)  # Coordonnée x
            y = d * math.sin(angle_rad)  # Coordonnée y
            pts.append([x, y, 0.0])  # Ajoute le point (z=0)
    return np.asarray(pts)  # Retourne un tableau numpy

# ***************************************************************
def icp_registration(source_pts, target_pts):
    """
    Effectue une registration ICP (Iterative Closest Point) entre deux nuages de points.

    ICP est une méthode pour aligner deux nuages de points en minimisant la distance entre eux.
    Ici, on utilise Open3D pour implémenter ICP avec un downsampling pour accélérer le calcul.

    Paramètres :
    - source_pts : Nuage de points source (à aligner)
    - target_pts : Nuage de points cible (référence)

    Retourne :
    - La matrice de transformation 4x4 pour aligner source sur target.
    """
    # Crée des nuages de points Open3D
    source = o3d.geometry.PointCloud()
    target = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(source_pts)
    target.points = o3d.utility.Vector3dVector(target_pts)

    # Downsampling pour réduire le bruit et accélérer ICP
    source = source.voxel_down_sample(0.05)  # Taille du voxel en mètres
    target = target.voxel_down_sample(0.05)

    # Paramètres ICP
    threshold = 0.2  # Distance maximale pour considérer un point comme correspondant
    init = np.eye(4)  # Matrice de transformation initiale (identité)

    # Exécute ICP
    reg = o3d.pipelines.registration.registration_icp(
        source, target, threshold, init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return reg.transformation  # Retourne la transformation calculée

# ***************************************************************
# Initialisation des variables globales
prev_scan = get_scan_points()  # Premier scan pour initialiser
pose = np.eye(4)  # Matrice de pose initiale (identité)
trajectory = []  # Liste pour stocker la trajectoire du LiDAR

scan_skip = 0  # Compteur pour sauter des scans
SKIP_N = 6  # Nombre de scans à sauter (réduit la dérive mais augmente le temps de calcul)
scan_index = 0  # Index pour nommer les fichiers de sauvegarde

# Carte cumulée pour ICP global (contient tous les points accumulés)
global_map = prev_scan.copy()

# Configuration de la visualisation Matplotlib en temps réel
plt.ion()  # Mode interactif
fig, ax = plt.subplots(figsize=(6,6))
scatter = ax.scatter(prev_scan[:,0], prev_scan[:,1], s=2, c='blue')  # Points du scan actuel
traj_line, = ax.plot([], [], 'g-', lw=1)  # Ligne pour la trajectoire
ax.set_aspect('equal')  # Échelle égale pour x et y
ax.set_xlim(-6, 6)  # Limites de l'affichage
ax.set_ylim(-6, 6)
ax.set_title('RPLidar 2D – ICP Point-to-Point Anti-Dérive')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.grid(True)  # Affiche une grille

# ***************************************************************
try:
    while True:  # Boucle principale
        scan_skip += 1
        curr_scan = get_scan_points()  # Récupère un nouveau scan

        # Saute des scans pour réduire la dérive (1 scan traité tous les SKIP_N scans)
        if scan_skip % SKIP_N != 0 or len(curr_scan) == 0:
            continue

        # ----- ICP point-to-point sur la carte cumulée -----
        # Calcule la transformation pour aligner le scan actuel sur la carte globale
        T = icp_registration(curr_scan, global_map)
        pose = pose @ T  # Met à jour la pose globale

        # Applique la transformation au scan actuel
        homog = np.hstack((curr_scan, np.ones((len(curr_scan),1))))  # Coordonnées homogènes
        curr_scan_tf = (pose @ homog.T).T[:, :3]  # Applique la transformation

        # ----- Mise à jour de la carte cumulée (avec downsampling) -----
        # Ajoute les nouveaux points à la carte globale
        global_map = np.vstack((global_map, curr_scan_tf))
        # Downsampling pour réduire la taille et le bruit
        pc_map = o3d.geometry.PointCloud()
        pc_map.points = o3d.utility.Vector3dVector(global_map)
        pc_map = pc_map.voxel_down_sample(0.05)
        global_map = np.asarray(pc_map.points)

        # ----- Loop closure simple : détection de boucles -----
        # Si la trajectoire est suffisamment longue, cherche des points proches
        if len(trajectory) > 50:
            tree = cKDTree(global_map[:,:2])  # Arbre KD pour une recherche rapide
            dist, idx = tree.query(curr_scan_tf[:,:2], k=1)  # Trouve le plus proche voisin
            close_points = dist < 0.5  # Points à moins de 0.5m
            if np.any(close_points):  # Si des points proches sont trouvés
                # Calcule la correction de position moyenne
                mean_target = np.mean(global_map[idx[close_points]], axis=0)
                mean_scan = np.mean(curr_scan_tf[close_points], axis=0)
                delta = mean_target - mean_scan
                pose[0:3,3] += delta  # Corrige la position

        # ----- Mise à jour de la trajectoire -----
        trajectory.append([pose[0,3], pose[1,3]])  # Ajoute la position actuelle

        # ----- Affichage Matplotlib -----
        # Met à jour l'affichage des points et de la trajectoire
        scatter.set_offsets(curr_scan_tf[:,:2])
        traj_line.set_data(np.array(trajectory)[:,0], np.array(trajectory)[:,1])
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        # ----- Sauvegarde du scan transformé -----
        # Enregistre le scan actuel dans un fichier texte
        filename = os.path.join(save_dir, f"scan_{scan_index:05d}.txt")
        np.savetxt(filename, curr_scan_tf, fmt="%.6f", delimiter=" ", header="x y z", comments="")
        scan_index += 1

        prev_scan = curr_scan  # Met à jour le scan précédent
        time.sleep(0.02)  # Pause pour limiter la fréquence

except KeyboardInterrupt:  # Arrêt manuel avec Ctrl+C
    print("Arrêt utilisateur")

finally:  # Nettoyage final
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
