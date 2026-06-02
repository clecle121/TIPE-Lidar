import os
import math

os.chdir('Position_ICP_Prof')

def lire_fichier_lidar(nom_fichier):
    angles, distances = [], []
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

alpha = 30  # secteur angulaire ± autour de 0°

fichier = "carte1_pol.txt"
angles, distances = lire_fichier_lidar(fichier)

with open('carte1_car.txt', 'w') as nv_fichier:
    for ang, dist in zip(angles, distances):
        rad = math.radians(ang)
        x = (dist * math.cos(rad)) / 1000
        y = dist * math.sin(rad) / 1000
        nv_fichier.write(f"{round(x, 2)} {round(y, 2)} 0\n")