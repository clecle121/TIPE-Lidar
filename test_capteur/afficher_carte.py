import os
import matplotlib.pyplot as plt
os.chdir('test_capteur\salle_de_classe')


def lire_fichier_cartesien(nom_fichier):
    xs, ys, zs = [], [], []
    with open(nom_fichier, "r") as f:
        for ligne in f:
            try:
                x, y, zs = ligne.split()
                xs.append(float(x))
                ys.append(float(y))
            except ValueError:
                continue
    return xs, ys

xs, ys = lire_fichier_cartesien("carte_car.txt")

plt.figure(figsize=(8, 8))
plt.scatter(xs, ys, s=5, color='steelblue')
plt.axis('equal')
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("Carte")
plt.grid(True, alpha=0.3)
plt.show()