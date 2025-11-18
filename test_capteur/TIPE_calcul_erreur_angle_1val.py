import math
import matplotlib.pyplot as plt
import numpy as np
import os
from TIPE_fonctions import *

os.chdir('test_capteur/valeurs_lidar') #Permet de changer le répertoire de travail. Ici je déplace le répertoire de travail dans le dossier "valeurs_lidar" pour avoir accès au fichier qui se trouve à l'intérieur.


# === Lecture du fichier ===
fichier = "test500cmV1.txt" #Nom du fichier que l'on veux étudier
angles, distances = lire_fichier_lidar(fichier)
points = polar_to_cartesian(angles, distances)


# Sélection des points autour de 0° (±10°)
angles_selection = []
points_selection = []
for ang, dist in zip(angles, distances): #Voir l'explication sur GPT pour le zip mais en vif permet de prendre un part un les éléments de 2 listes pour en faire des trucs.
    if -10 <= ang <= 10 or ang >= 350: #Permet de ne prendre en compte que les valeurs autour de ±10°
        rad = math.radians(ang) #Convertit les angles normalement en ° en rad
        x = dist * math.cos(rad)
        y = dist * math.sin(rad)
        angles_selection.append(ang)
        points_selection.append((x, y))


# Régression linéaire sur ces points
x_sel, y_sel = zip(*points_selection)
a, b = np.polyfit(x_sel, y_sel, 1)  # y = a*x + b
print(f"Équation de la droite du mur : y = {a:.3f}x + {b:.3f}")

# Génération des points de la droite pour le tracé
x_min, x_max = min(x_sel), max(x_sel)
x_line = np.linspace(x_min, x_max, 100)
y_line = a * x_line + b


# équation de la droite perpendiculaire à la regression linéaire passant par (0, 0)
m = -1/a #Pour des doites perpendiculaires le produit de leur coefficients directeur fait -1 et on sait que la droite qu'on charche passe pas (0, 0) donc b = 0.

#
x_perpen = []
y_perpen = []
for i in range(int((-b)/(a-m))+1):
    x_perpen.append(i)
    y_perpen.append(m * i)  

#centre = (0, 0) #pas d'utiliter pour le moment. tracer l'angle sur le graphique.


# === Affichage graphique ===
x_vals, y_vals = zip(*points)
plt.figure(figsize=(8,8))
plt.scatter(x_vals, y_vals, c="red", s=5, label="Points LIDAR") #"c" permet de modifier les couleurs des points et "s" permet de modifier la taille des points.
plt.scatter(x_sel, y_sel, c="blue", s=10, label="Points mur")
plt.plot(x_line, y_line, "g-", linewidth=2, label="Mur estimé")
plt.plot(x_perpen, y_perpen, "m-", linewidth=2, label="droite perpend au mur")
plt.axhline(0, color="black", linewidth=0.5) #permet de placer un ligne horizontale sur le graphique
plt.axvline(0, color="black", linewidth=0.5) #permet de placer un ligne verticale sur le graphique
plt.gca().set_aspect("equal", adjustable="datalim") #permet de faire en sorte que les axes soient équivalent


# Ajouter l’équation sur le graphique
plt.text(
    0.05, 0.95,
    f"Mur : y = {a:.3f}x + {b:.3f}",
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