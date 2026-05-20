import serial
import time
import struct
import numpy as np
import matplotlib.pyplot as plt

"""
================================================================================
                    LiDAR D500 - Gestion du passage 360°→0°
================================================================================

DESCRIPTION :
-------------
Ce script se connecte à un LiDAR D500 via un port série, capture les données,
et les affiche toutes les 100 rotations complètes.
- Les angles sont ajustés pour éviter les sauts (ex: 355° → 365° au lieu de 5°).
- Tous les points sont affichés, même après 350°.
- Pour arrêter, appuyer sur 'l' dans la fenêtre du graphique.

UTILISATION :
-------------
1. Brancher le LiDAR en USB et identifier le port série (ex: /dev/ttyUSB0 ou COM3).
2. Mettre à jour la variable PORT avec le bon port.
3. Exécuter le script. Pour arrêter, cliquer sur la fenêtre du graphique et appuyer sur 'l'.
"""

# =============================================================================
#                           CONFIGURATION INITIALE
# =============================================================================

PORT = "/dev/ttyUSB0"  # Port série du LiDAR
BAUDRATE = 230400      # Vitesse de transmission en bauds
MAGIC = 0x54          # Octet magique pour identifier le début d'une trame
N_PTS = 12            # Nombre de points de mesure par trame
PKT_LEN = 47          # Longueur totale d'une trame en octets

# =============================================================================
#                           FONCTIONS
# =============================================================================

def parse_frame(frame: bytes):
    """
    Décode une trame brute de 47 octets en une liste de dictionnaires.
    """
    if len(frame) < PKT_LEN or frame[0] != MAGIC:
        return None

    speed = struct.unpack_from('<H', frame, 2)[0] / 100.0
    start_a = struct.unpack_from('<H', frame, 4)[0] / 100.0
    end_a = struct.unpack_from('<H', frame, 42)[0] / 100.0
    ts = struct.unpack_from('<H', frame, 44)[0]

    points = []
    for i in range(N_PTS):
        off = 6 + i * 3
        dist = struct.unpack_from('<H', frame, off)[0]
        inty = frame[off + 2]
        angle = start_a + (end_a - start_a) * i / (N_PTS - 1)
        points.append({
            "angle": round(angle, 2),
            "dist_mm": dist,
            "intensity": inty,
            "speed": round(speed, 1),
            "ts_ms": ts,
        })
    return points

# =============================================================================
#                           INITIALISATION DU GRAPHIQUE
# =============================================================================

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='polar')
ax.set_theta_zero_location('N')  # 0° en haut (Nord)
ax.set_theta_direction(-1)       # Sens horaire (comme un LiDAR)
ax.set_rmax(2000)                 # Distance maximale en mm
ax.set_title("LiDAR D500 - Gestion du passage 360°→0°\n(Appuyez sur 'l' pour quitter)", pad=20)
scatter = ax.scatter([], [], c=[], cmap='viridis', s=10, alpha=0.7)
plt.tight_layout()

# Variable pour gérer la fermeture
stop_program = False

# Fonction pour gérer les événements clavier
def on_key_press(event):
    global stop_program
    if event.key == 'l':
        stop_program = True
        print("\n[INFO] Touche 'l' pressée. Fermeture en cours...")

# Connexion de l'événement clavier
fig.canvas.mpl_connect('key_press_event', on_key_press)

# =============================================================================
#                           PROGRAMME PRINCIPAL
# =============================================================================

print("==================================================")
print("  D500 LiDAR -- Gestion du passage 360°→0°")
print("==================================================")

# Initialisation de la connexion série
try:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
    )
    print(f"[OK] Port {PORT} ouvert à {BAUDRATE} bauds.")
except serial.SerialException as e:
    print(f"[ERREUR] Impossible d'ouvrir {PORT} : {e}")
    raise

ser.reset_input_buffer()
time.sleep(0.2)

# Variables pour la capture
buf = bytearray()
all_angles = []
all_dists = []
all_intensities = []
rotation_count = 0  # Compteur de rotations complètes
TOTAL_ROTATIONS = 100  # Nombre de rotations avant affichage
last_angle = None  # Dernier angle traité (ajusté si nécessaire)

def adjust_angle(current_angle, last_angle):
    """
    elimine les sauts d'angle lors du passage de 360° à 0°. 
    plus précisément les valeurs problématiques
    """
    var = 0
    if last_angle is None:
        return current_angle
    if var != 0 :
        var -= 1
        return last_angle + 0.8
    if current_angle < last_angle and last_angle > 200 :
        var = 10
        return last_angle + 0.8
    return current_angle

# Fonction pour mettre à jour le graphique
def update_plot():
    """Met à jour le graphique avec les données accumulées."""
    if len(all_angles) > 0:
        # Convertir les angles ajustés en radians pour matplotlib
        theta = np.deg2rad(all_angles)
        scatter.set_offsets(np.column_stack((theta, all_dists)))
        scatter.set_array(all_intensities)
        plt.draw()

# Boucle principale de capture
try:
    print("[INFO] Appuyez sur 'l' dans la fenêtre du graphique pour arrêter...")
    plt.show(block=False)  # Affiche la fenêtre sans bloquer
    while not stop_program:
        waiting = ser.in_waiting
        if waiting > 0:
            chunk = ser.read(waiting)
            buf.extend(chunk)

            # Extraction des trames complètes
            while True:
                idx = buf.find(MAGIC)
                if idx == -1:
                    buf.clear()
                    break
                if idx > 0:
                    buf = buf[idx:]
                if len(buf) < PKT_LEN:
                    break

                frame = bytes(buf[:PKT_LEN])
                buf = buf[PKT_LEN:]
                result = parse_frame(frame)

                if result:
                    # Traiter chaque point de la trame
                    for p in result:
                        if 0 <= p['dist_mm'] <= 35:
                            continue

                        current_angle = p['angle']
                        adjusted_angle = adjust_angle(current_angle, last_angle)

                        # Mettre à jour last_angle avec l'angle ajusté
                        last_angle = adjusted_angle

                        # Ajouter le point ajusté aux listes
                        all_angles.append(adjusted_angle)
                        all_dists.append(p['dist_mm'])
                        all_intensities.append(p['intensity'])

                    rotation_count += 1  # Incrémente le compteur de rotations

                    # Afficher toutes les TOTAL_ROTATIONS rotations
                    if rotation_count >= TOTAL_ROTATIONS:
                        update_plot()
                        # Réinitialiser les données pour les prochaines rotations
                        all_angles = []
                        all_dists = []
                        all_intensities = []
                        rotation_count = 0
                        last_angle = None  # Réinitialiser pour le prochain bloc
        else:
            time.sleep(0.001)
            plt.pause(0.001)  # Permet de traiter les événements de la fenêtre

except KeyboardInterrupt:
    pass

# Fermeture propre
print("\n[INFO] Fermeture en cours...")
ser.close()
plt.close()
print("[OK] Port série fermé.")
print("[OK] Fenêtre graphique fermée.")
exit(0)
