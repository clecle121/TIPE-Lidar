# -*- coding: utf-8 -*-
import math
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from sklearn.linear_model import LinearRegression

import os
os.environ["OMP_NUM_THREADS"] = "1"
os.chdir('test_capteur/valeurs 2')


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
    
def traiter_les_coo(txt):
    don = []
    angles, dista = lire_fichier_lidar(txt)
    lis_x = []; lis_y = []
    
    for ang, dis in zip(angles, dista):
        for i in range(len(angles)):
            x, y = polar_to_cartesian(angles[i], dista[i])
            lis_x.append(x)
            lis_y.append(y)
            don.append([x,y])
    return don

txt = "Lidar20V1.txt"
donnees = traiter_les_coo(txt)

donnees = np.array(donnees)

# Application de K-means
kmeans = KMeans(n_clusters=6, random_state=0).fit(donnees)
labels = kmeans.labels_

# Régression linéaire pour chaque cluster
droites = []
plt.figure(figsize=(10, 6))
plt.scatter(donnees[:, 0], donnees[:, 1], c=labels, cmap='viridis')

for i in range(4):
    cluster = donnees[labels == i]
    if len(cluster) > 1:
        reg = LinearRegression().fit(cluster[:, 0].reshape(-1, 1), cluster[:, 1])
        droites.append((reg.coef_[0], reg.intercept_))
        x = np.linspace(min(donnees[:, 0]), max(donnees[:, 0]), 100)
        y = reg.predict(x.reshape(-1, 1))
        plt.plot(x, y, '-r', label=f"Droite {i+1}: y = {reg.coef_[0]:.2f}x + {reg.intercept_:.2f}")

plt.title("Droites identifiées par K-means et régression linéaire")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.legend()
plt.show()

# Affichage des équations des droites
for idx, (pente, ordonnee) in enumerate(droites, 1):
    print(f"Droite {idx}: y = {pente:.2f}x + {ordonnee:.2f}")