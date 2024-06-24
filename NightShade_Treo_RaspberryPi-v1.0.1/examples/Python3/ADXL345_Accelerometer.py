"""
ADXL345_Accelerometer - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo ADXL345 accelerometer module. (NSE-1123-1) It it reads the
accelerations measured by the sensor and prints them over
serial at 115200 baudrate.

Created by Aaron D. Liebold
on February 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page: https://nightshade.net/product/treo-3-axis-accelerometer-adxl345/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

# Include NightShade Treo Library
import NightShade_Treo
from time import sleep

# Declare Objects
accel = NightShade_Treo.ADXL345(1)

accel.begin()
accel.setMeasurementRange(0)

while True:
    accel.retrieveData()
    X = accel.readX()
    Y = accel.readY()
    Z = accel.readZ()
    print("X:" + str(X) + " Y:" + str(Y) + " Z:" + str(Z))
    sleep(0.25)
