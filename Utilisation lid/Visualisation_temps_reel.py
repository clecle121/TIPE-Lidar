import serial
import math
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
from collections import deque

# Configuration du port série (à adapter selon votre configuration)
SERIAL_PORT = 'COM7'  # Remplacez par le port série correct
BAUD_RATE = 115200    # À adapter selon la documentation du lidar
MAX_RANGE = 10.0      # Rayon maximum en mètres
MAX_POINTS = 2000     # Nombre maximum de points à afficher

# Initialisation de la connexion série
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connecté au port {SERIAL_PORT} à {BAUD_RATE} bauds.")
except serial.SerialException as e:
    print(f"Erreur de connexion au port série: {e}")
    ser = None

# Initialisation des données pour l'affichage
points = deque(maxlen=MAX_POINTS)

# Configuration de la fenêtre PyQtGraph
app = QtGui.QApplication([])
win = pg.GraphicsLayoutWidget(title="Affichage des points du Lidar en temps réel")
win.resize(800, 800)
win.setWindowTitle('Affichage des points du Lidar en temps réel')

# Création d'un graphique
plot = win.addPlot()
plot.setXRange(-MAX_RANGE, MAX_RANGE)
plot.setYRange(-MAX_RANGE, MAX_RANGE)
plot.setLabel('left', 'Y (m)')
plot.setLabel('bottom', 'X (m)')
plot.showGrid(x=True, y=True)

# Initialisation des données pour le graphique
scatter = plot.plot([], [], pen=None, symbol='o', symbolSize=2)

# Fonction pour mettre à jour l'affichage
def update():
    if ser:
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                try:
                    # Exemple de traitement des données (à adapter selon le format réel)
                    data = line.split(',')
                    if len(data) == 2:
                        angle, distance = map(float, data)
                        if distance <= MAX_RANGE:
                            # Conversion en coordonnées cartésiennes
                            x = distance * math.cos(math.radians(angle))
                            y = distance * math.sin(math.radians(angle))
                            points.append((x, y))
                except (ValueError, AttributeError, IndexError):
                    pass

    if points:
        x_vals, y_vals = zip(*points)
        scatter.setData(x=x_vals, y=y_vals)

# Timer pour mettre à jour l'affichage
timer = QtGui.QTimer()
timer.timeout.connect(update)
timer.start(50)  # Mise à jour toutes les 50 ms

# Affichage de la fenêtre
win.show()

# Démarrage de l'application
QtGui.QApplication.instance().exec_()

# Fermez le port série à la fin
if ser:
    ser.close()
