# -*- coding: utf-8 -*-
"""
Created on Fri Oct  3 14:12:37 2025

@author: clemc
"""
import math
import matplotlib.pyplot as plt
import numpy as np

def lire_fichier_lidar(nom_fichier):
    angles = []
    distances = []
    with open(nom_fichier, "r") as f:
        for ligne in f:
            if ligne.startswith("#") or "Angule" in ligne:
                continue
            try:
                angle, distance, quality = ligne.split()
                angles.append(float(angle))
                distances.append(float(distance))
            except ValueError:
                continue
    return angles, distances

"""
def polar_to_cartesian(angles, distances):
    x_coords = []
    y_coords = []
    for angle, dist in zip(angles, distances):
        rad = math.radians(angle)
        x = dist * math.cos(rad)
        y = dist * math.sin(rad)
        x_coords.append(x)
        y_coords.append(y)
    return x_coords, y_coords
"""

def polar_to_cartesian(angles, distances):
    if isinstance(angles, float) and isinstance(distances, float):
        rad = math.radians(angles)
        x = distances * math.cos(rad)
        y = distances * math.sin(rad)
        return x, y
    else:
        x_coords = []
        y_coords = []
        for angle, dist in zip(angles, distances):
            rad = math.radians(angle)
            x = dist * math.cos(rad)
            y = dist * math.sin(rad)
            x_coords.append(x)
            y_coords.append(y)
        return x_coords, y_coords


# === Lecture du fichier ===
def lecture_fichier(fichier):
    angles, distances = lire_fichier_lidar(fichier)
    
    lis_x = []; lis_y = []
    for ang, dist in zip(angles, distances):
        if -15 <= ang <= 15 or ang >= 345:
            x, y = polar_to_cartesian(ang, dist)
            lis_x.append(x)
            lis_y.append(y)
    a = 0; b = 0       
    droite = np.polyfit(lis_x, lis_y, 1)  # y = a*x + b
    a, b = droite
    
    # === Calcul de la distance Lidar mur ===
    
    d = math.fabs(b) / ((a**2 + 1)**0.5)
    
    print(f"Mur : y = {a:.3f}x + {b:.3f}")
    print(f"Dist = {d:.3f}")
    return float(f"{d:.2f}")


dists = [50, 112, 200, 300, 500]
nb = [1, 2, 3, 4, 5]
def noms_fichiers(dists,nb):
    txt = []
    for di in range(len(dists)):
        for n in range(len(nb)):
            txt.append(f"test{dists[di]}cmV{nb[n]}.txt")  #"test50cmV1.txt
    return txt


txt = noms_fichiers(dists,nb)
dist_fin = []
for i in range(len(txt)):
    dist_fin.append(lecture_fichier(txt[i]))


print(dist_fin)















"""
plt.figure(figsize=(8,8))
plt.scatter(lis_x, lis_y, c="red", s=5, label="Points LIDAR")
#plt.scatter(x_sel, y_sel, c="blue", s=10, label="Points mur")
plt.plot(x_line, y_line, "g-", linewidth=2, label="Mur estimé")

plt.text(
    0.05, 0.95,
    f"dist_mur_dist : {d:.3f}",
    transform=plt.gca().transAxes,
    fontsize=10,
    verticalalignment="top",
    bbox=dict(facecolor="white", alpha=0.7, edgecolor="black")
)

plt.title("Nuage de points LIDAR + Droite du mur estimée")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.legend()
plt.grid(True)
plt.show()
"""