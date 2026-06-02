from rplidar import RPLidar
import keyboard
import time

# =========================
# CONFIGURATION
# =========================

PORT = "COM5"

# =========================
# INITIALISATION
# =========================

lidar = RPLidar(PORT, baudrate=115200)

print("Connexion au lidar...")
print(lidar.get_info())
print(lidar.get_health())

lidar.stop_motor()
lidar.stop()

print("\nTourelle arrêtée.")
print("ESPACE = scan 360° | Q = quitter\n")

try:

    while True:

        if keyboard.is_pressed('q'):
            break

        if keyboard.is_pressed('space'):

            print("\nDémarrage moteur...")

            # purge buffer (TA VERSION)
            lidar.clear_input()

            lidar.start_motor()
            time.sleep(2)

            print("Scan 360° en cours...")

            mesures = []
            previous_angle = None

            started = False
            completed = False

            # =========================
            # ACQUISITION
            # =========================

            for _, quality, angle, distance in lidar.iter_measurments():

                wrap = False

                if previous_angle is not None:
                    wrap = previous_angle > 300 and angle < 60

                # début vrai tour
                if wrap and not started:
                    mesures = []
                    started = True
                    print("Début vrai tour 360°")

                # fin du tour
                elif wrap and started:
                    completed = True
                    break

                if started:
                    mesures.append((angle, distance))

                previous_angle = angle

            # =========================
            # RESULTATS
            # =========================

            print(f"\nNombre de points : {len(mesures)}")

            for angle, distance in mesures[:10]:
                print(f"{angle:.2f}°  {distance:.1f} mm")

            lidar.stop()
            lidar.stop_motor()

            print("\nScan terminé. Tourelle arrêtée.\n")

            while keyboard.is_pressed('space'):
                time.sleep(0.1)

        time.sleep(0.05)

except KeyboardInterrupt:
    pass

finally:

    print("\nArrêt du lidar...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

    print("Déconnexion OK.")