from rplidar import RPLidar
import os
from math import floor
import numpy as np

lidar = RPLidar('/dev/ttyUSB0')

scan_data = [0]*360

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

def getDist():
    minDist = 0
    lidar = RPLidar('/dev/ttyUSB0')
    try:
        for i, scan in enumerate(lidar.iter_scans()):
            print('%d: Got %d measurments' % (i, len(scan)))
            for (_, angle, distance) in scan:
                    # print("angle:",angle)
                    scan_data[min([359, floor(angle)])] = distance
            # print(scan_data)
            allDists = [scan_data[i] for i in range(360)
                                        if i >= 31 and i <= 62 and scan_data[i] > 0]
        # minDist = np.median(allDists)
        # print(minDist)
            if (2 * len(allDists) > 62 - 31):
                minDist = np.median(allDists)
                print("min:",minDist)
                lidar.stop()
                lidar.disconnect()
                return 

    except KeyboardInterrupt:
        print("Stoping LIDAR Scan")

while (1):
    print("t=",getDist())