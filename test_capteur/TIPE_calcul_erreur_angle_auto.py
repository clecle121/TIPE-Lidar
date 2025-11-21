import math
import matplotlib.pyplot as plt
import numpy as np
import os
from TIPE_fonctions import *

os.chdir('test_capteur/valeurs_lidar')


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
def lecture_fichier(fichier,x):
    angles, distances = lire_fichier_lidar(fichier)
    alpha = math.degrees(np.arctan(650/(x*10+35)))
    lis_x = []; lis_y = []
    for ang, dist in zip(angles, distances):
        if -alpha <= ang <= alpha or ang >= 360-alpha :
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

dists = [20,30,40,50,55,70,80,90,100,112,130,140,150,175,190,200,220,250,275,300,320,340,360,380,400,450,500]
nb = [1, 2, 3, 4, 5]

def noms_fichiers(dists,nb):
    txt = []
    for di in range(len(dists)):
        for n in range(len(nb)):
            txt.append(f"test{dists[di]}cmV{nb[n]}.txt")  #"test50cmV1.txt"
    return txt


txt = noms_fichiers(dists,nb)
dist_fin = []
for i in range(len(txt)):
    dist_fin.append(lecture_fichier(txt[i],dists[i//5]))
print(dist_fin)