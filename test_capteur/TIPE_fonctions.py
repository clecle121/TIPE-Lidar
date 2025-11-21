import math


def lire_fichier_lidar(nom_fichier):
    '''
    Permet de récuperer seulement les angles et les distances dans un fichier d'aquisition de mesure du capteur lidar.

    Entrée :
    nom_fichier: (str) correspond au nom du fichier dont on veut récuperer les valeurs.

    Sortie :
    (list with float) renvoie deux listes, une appelé "angles" et l'autre "distances".
    '''
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

def noms_fichiers(list_distances,numero):
    '''
    Permet de créer et stocker rapidement le nom de tout les fichiers de mesure du lidar.

    Entrée :
    list_distances: (list with int) contient les différentes distances de mesure faite avec le lidar.
    numero: (list with int) contient des nombres de 0 jusqu'à x. x correspond au nombre de mesure effectué au total pour une seul distance.

    Sortie :
    (list with txt) renvoie une liste appelée "txt". Elle contient les noms des fichiers de mesures.
    '''
    txt = []
    for distance in range(len(list_distances)):
        for i in range(len(numero)):
            txt.append(f"test{list_distances[distance]}cmV{numero[i]}.txt")  #"test50cmV1.txt"
    return txt


#Il y a deux polar_to_cartesian différent lequel est le bon ???
#celui ci est présent seulement dans TIPE_Calcul_erreur_dist_auto
"""
def polar_to_cartesian(angles, distances):
    '''
    '''
    if isinstance(angles, float) and isinstance(distances, float): #??? "isinstance"
        rad = math.radians(angles)
        x = distances * math.cos(rad)
        y = distances * math.sin(rad)
        return x, y
    else:
        x_coords = []
        y_coords = []
        for angle, dist in zip(angles, distances): #Expliquer le "zip"
            rad = math.radians(angle)
            x = dist * math.cos(rad)
            y = dist * math.sin(rad)
            x_coords.append(x)
            y_coords.append(y)
        return x_coords, y_coords
"""

def polar_to_cartesian(angles, distances):
    '''
    '''
    points = []
    for angle, dist in zip(angles, distances):
        rad = math.radians(angle)
        x = dist * math.cos(rad)
        y = dist * math.sin(rad)
        points.append((x, y))
    return points


def lecture_fichier(fichier):
    '''
    '''
    angles, distances = lire_fichier_lidar(fichier)
    
    lis_x = []; lis_y = []
    for ang, dist in zip(angles, distances):
        if -15 <= ang <= 15 or ang >= 345:
            x, y = polar_to_cartesian(ang, dist)
            lis_x.append(x)
            lis_y.append(y)      
    a, b = np.polyfit(lis_x, lis_y, 1)