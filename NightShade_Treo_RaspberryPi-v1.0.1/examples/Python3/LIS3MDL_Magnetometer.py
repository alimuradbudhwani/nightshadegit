"""
LIS3MDL_Magnetometer - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo LIS3MDL magnetometer module. (NSE-1126-1)
It prints the magnetometer data, a calculated heading,
and temperature from the sensor and prints in out as
Serial at 115200 baudrate.

Created by Aaron D. Liebold
on February 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page: https://nightshade.net/product/treo-3-axis-magnetometer-lis3mdl/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

# Include NightShade Treo Library
import NightShade_Treo
from time import sleep
import math

# Declare Objects
sensor = NightShade_Treo.LIS3MDL(1)
sensor.begin()

while True:
    sensor.acquireMagData()
    X = sensor.readX() / sensor.oneGaussValue()     # X value in Gauss
    Y = sensor.readY() / sensor.oneGaussValue()     # Y value in Gauss
    Z = sensor.readZ() / sensor.oneGaussValue()    # Z value in Gauss
    temp = sensor.readTemp() / 10    # Retrieve temperature in deg C

    # Calculate Azimuth at X+ (2-axis)
    azimuth = math.atan2(Y, X) * 180 / math.pi
    if azimuth < 0:
        azimuth += 360.0    # Keep azimuth positive (0 - 360deg)

    print("Az: " + str(round(azimuth, 2)) + "deg \t" +
          str(round(X, 2)) + "G\t" + str(round(Y, 2)) +
          "G\t" + str(round(Z, 2)) + "G\t" + str(round(temp, 2)) + "C\t")

    sleep(0.5)
