# -*- coding: utf-8 -*-
"""
Created on Thu May 27 12:01:20 2021

@author: nicov
"""
import os
os.environ["OMP_NUM_THREADS"] = "1"
import math
import numpy as np
import matplotlib.pyplot as plt

from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

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
            don.append([lis_x,lis_y])
    return don


######### Valeurs

txt = f"Valeurs Lidar/test500cmV5.txt"
donnees = traiter_les_coo(txt)
K= 2
KM=KMeans(n_clusters=K, random_state=170)

labels_predits = KM.fit_predict(donnees)

#affichage des nuages de points colorés en fonction du label prédit
import matplotlib.pyplot as plt

Donnees_par_classe=[]
marqueurs=['.','*','o','v','^','d','+']
fig, ax = plt.subplots()

L_type_de_classes=np.unique(labels_predits)
nb_classes=len(L_type_de_classes)
for classe in L_type_de_classes:
    X=[]
    Y=[]
    for i in range(len(donnees)):
        if classe ==labels_predits[i]:
            X.append(donnees[i][0])
            Y.append(donnees[i][1])
    Donnees_par_classe.append([X,Y])
    ax.plot(X,Y,marqueurs[classe],label='classe '+str(classe))

ax.plot(KM.cluster_centers_[:,0],KM.cluster_centers_[:,1],c='r')
ax.set_xlabel('Longueur du bec')
ax.set_ylabel('Profondeur du bec')
ax.legend()