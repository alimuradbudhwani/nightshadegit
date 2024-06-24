"""
ICM20948_9DoF_IMU - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo ICM20948 9 DoF IMU module. (NSE-1124-1)
It reads the accelerometer, gyro, and magnetometer data
from the sensor and prints it over Serial at 115200
baudrate.

Created by Aaron D. Liebold
on February 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page: https://nightshade.net/product/treo-9-dof-imu-icm20948/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

 # Include NightShade Treo Library
import NightShade_Treo
import math
from time import sleep

#include <math.h>

 # Declare Objects
sensor = NightShade_Treo.ICM20948(1)
#sensor.begin()

sensor.enableSleep(0) # Wake ICM20948
sensor.enableI2cPassthrough(1)
sensor.magBegin()

while True:
    sensor.getData(1, 1)
    accelX = sensor.readAccelX() / sensor.accelSensitivity()    # G
    accelY = sensor.readAccelY() / sensor.accelSensitivity()    # G
    accelZ = sensor.readAccelZ() / sensor.accelSensitivity()    # G
    gyroX = sensor.readGyroX() / sensor.gyroSensitivity()   # dps
    gyroY = sensor.readGyroY() / sensor.gyroSensitivity()   # dps
    gyroZ = sensor.readGyroZ() / sensor.gyroSensitivity()   # dps
    magX = (sensor.readCompassX() * 15) / 100   # uT
    magY = (sensor.readCompassY() * 15) / 100   # uT
    magZ = (sensor.readCompassZ() * 15) / 100   # uT

    # Calculate Azimuth at X+ (2-axis)
    azimuth = math.atan2(-1*magY, magX) * 180 / math.pi
    if azimuth < 0:
        azimuth += 360.0    # Keep azimuth positive (0 - 360deg)


    # Print Results
    print("Accel X: " + str(accelX) + "g \tY: " + str(accelY) + "g \tZ: " + str(accelZ) + "g\tGyro X: "
    + str(gyroX) + "dps \tY: " + str(gyroY) + "dps \tZ: " + str(gyroZ) + "dps\tCompass Heading: "
    + str(round(azimuth, 2)) + "deg")

    sleep(0.5)
