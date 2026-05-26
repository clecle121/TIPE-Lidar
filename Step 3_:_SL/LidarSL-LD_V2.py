import serial
import time
import os
import struct
import re

PORT     = "/dev/ttyUSB0"
BAUDRATE = 230400
t        = 2
OUTPUT   = "Test_3.txt"

MAGIC    = 0x54
N_PTS    = 12
PKT_LEN  = 47  # 1 magic + 1 nb_pts + 2 speed + 2 start_angle
               # + 12*(2 dist + 1 intensity) + 2 end_angle + 2 timestamp + 1 CRC
# Ce script lit le LiDAR D500 via le port serie, decode des trames de 47 octets
# et enregistre chaque point mesure (angle, distance, intensite, vitesse, timestamp)
# dans un fichier texte. La lecture s'arrete proprement apres t secondes d'acquisition.

def parse_frame(frame: bytes):
    """Décode une trame D500 (47 bytes) -> liste de dicts."""
    if len(frame) < PKT_LEN or frame[0] != MAGIC:
        return None
    speed   = struct.unpack_from('<H', frame, 2)[0] / 100.0   # deg/s
    start_a = struct.unpack_from('<H', frame, 4)[0] / 100.0   # degrés
    end_a   = struct.unpack_from('<H', frame, 42)[0] / 100.0  # degrés
    ts      = struct.unpack_from('<H', frame, 44)[0]          # ms
    points  = []
    for i in range(N_PTS):
        off  = 6 + i * 3
        dist = struct.unpack_from('<H', frame, off)[0]        # mm
        inty = frame[off + 2]                                 # 0-255
        angle = start_a + (end_a - start_a) * i / (N_PTS - 1)
        points.append({
            "angle":     round(angle, 2),
            "dist_mm":   dist,
            "intensity": inty,
            "speed":     round(speed, 1),
            "ts_ms":     ts,
        })
    return points

def adjust_angle(current_angle, last_angle, compteur):
    """
    Gère les sauts d'angle lors du passage de 360° à 0°.
    Renvoie toujours un angle ajusté et la valeur mise à jour de compteur.
    """
    if last_angle is None:
        return current_angle, compteur
    if compteur != 0:
        return last_angle + 0.8, compteur - 1
    if current_angle < last_angle and last_angle > 200:
        return last_angle + 0.8, 9
    return current_angle, compteur

print("==================================================")
print("  D500 LiDAR -- Enregistreur de donnees")
print("==================================================")

try:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
    )
    print("[OK] Port " + PORT + " ouvert a " + str(BAUDRATE) + " baud.")
except serial.SerialException as e:
    print("[ERREUR] Impossible d'ouvrir " + PORT + " : " + str(e))
    raise

ser.reset_input_buffer()
time.sleep(0.2)

print("[INFO] Enregistrement pendant " + str(t) + " s -> " + OUTPUT + " ...")

buf         = bytearray()
all_points  = []
total_bytes = 0
start_time  = time.time()
last_angle  = None
compteur    = 0
all_angles = []
all_dists = []
all_intensities = []
rotation_count = 0

while time.time() - start_time < t:
    waiting = ser.in_waiting
    if waiting > 0:
        chunk = ser.read(waiting)
        buf.extend(chunk)
        total_bytes += len(chunk)
        # Extraire toutes les trames complètes disponibles dans le buffer
        while True:
            idx = buf.find(MAGIC)
            if idx == -1:
                buf.clear()
                break
            if idx > 0:
                buf = buf[idx:]          # aligner sur le magic byte
            if len(buf) < PKT_LEN:
                break                    # pas encore assez de données
            frame  = bytes(buf[:PKT_LEN])
            buf    = buf[PKT_LEN:]
            result = parse_frame(frame)
            if result:
                # Traiter chaque point de la trame en comparant avec la valeur precedente
                for p in result:
                    adjusted_angle, compteur = adjust_angle(p['angle'], last_angle, compteur)
                    p['angle'] = adjusted_angle
                    last_angle = adjusted_angle
                    all_points.append(p)
    else:
        time.sleep(0.001)

ser.close()

with open(OUTPUT, "w", encoding="utf-8") as f:
    f.write(f"{'angle_deg':>10}  {'dist_mm':>8}  {'intensity':>9}  {'speed_dps':>9}  {'ts_ms':>7}\n")
    f.write("-" * 55 + "\n")
    for p in all_points:
        f.write(
            f"{p['angle']:>10.2f}  {p['dist_mm']:>8}  {p['intensity']:>9}  "
            f"{p['speed']:>9.1f}  {p['ts_ms']:>7}\n"
        )

size_kb = total_bytes / 1024
print("[OK] Termine. " + str(total_bytes) + " octets bruts recus (" + str(round(size_kb, 2)) + " Ko).")
print("[OK] " + str(len(all_points)) + " points ecrits -> " + os.path.abspath(OUTPUT))
print("[INFO] Port " + PORT + " ferme.")

