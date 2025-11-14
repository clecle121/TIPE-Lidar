def lire_fichier_lidar(nom_fichier):
    '''
    Entrée :
    Sortie : 
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


def noms_fichiers(dists,nb):
    '''
    '''
    txt = []
    for di in range(len(dists)):
        for n in range(len(nb)):
            txt.append(f"test{dists[di]}cmV{nb[n]}.txt")  #"test50cmV1.txt
    return txt


def polar_to_cartesian(angles, distances):
    '''
    '''
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