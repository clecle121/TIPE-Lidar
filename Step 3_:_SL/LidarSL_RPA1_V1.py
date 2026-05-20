from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'
PROCESS_EVERY_NTH_SCAN = 100
ANGLE_MIN = 80.0
ANGLE_MAX = 100.0


def filter_scan(scan):
    return [
        (quality, angle, distance)
        for quality, angle, distance in scan
        if ANGLE_MIN <= angle <= ANGLE_MAX
    ]


def run():
    lidar = RPLidar(PORT_NAME)
    first_filtered = None
    scan_count = 0

    try:
        for scan in lidar.iter_scans():
            scan_count += 1
            if scan_count % PROCESS_EVERY_NTH_SCAN != 0:
                continue

            filtered = filter_scan(scan)
            if first_filtered is None:
                first_filtered = filtered
                print(
                    f"Premier tour #{scan_count} enregistré, {len(filtered)} points entre {ANGLE_MIN}° et {ANGLE_MAX}°"
                )
            else:
                print(
                    f"Tour #{scan_count} : {len(filtered)} points entre {ANGLE_MIN}° et {ANGLE_MAX}°"
                )

            for quality, angle, distance in filtered:
                print(f"  angle={angle:.2f} deg dist={distance:.1f} mm q={quality}")
    
    except KeyboardInterrupt:
        print('Arrêt demandé')
    finally:
        lidar.stop()
        lidar.disconnect()


if __name__ == '__main__':
    run()
