import matplotlib.pyplot as plt
import numpy as np

# Lire le fichier
file_path = '/home/cam/Documents/Codes VS/TIPE-Lidar/Test_3.txt'

# Lister pour stocker les valeurs
theta = []
rho = []

# Lire et parser le fichier
with open(file_path, 'r') as file:
    for line in file:
        line = line.strip()
        if line:  # Ignorer les lignes vides
            parts = line.split()
            if len(parts) >= 2:
                try:
                    theta.append(float(parts[0]))
                    rho.append(float(parts[1]))
                except ValueError:
                    continue

# Afficher les points polaires
fig, ax = plt.subplots(subplot_kw=dict(projection='polar'))
ax.plot(theta, rho, 'o', markersize=5)
ax.set_title('Points Polaires')
plt.show()
