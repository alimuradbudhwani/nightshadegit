"""
HDC1080_TempHumidity - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo HDC1080 temperature and humidity sensing
module. (NSE-1135-1) It prints the temperature and
humidity values to Serial at 115200 baudrate.

Created by Aaron D. Liebold
on February 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page: https://nightshade.net/product/treo-temp-humidity-sensor-hdc1080/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

# Include NightShade Treo Library
import NightShade_Treo
from time import sleep

sensor = NightShade_Treo.HDC1080(1)

sensor.begin()
sensor.setTemperatureResolution(0)
sensor.setHumidityResolution(0)

deviceID = sensor.readDeviceId()
mfgID = sensor.readManufacturerId()
serialID = sensor.readSerialId()

print("Device ID: " + hex(deviceID))
print("Manufacturer ID: " + hex(mfgID))
print("Serial ID: " + hex(serialID))

sleep(0.001)

while True:
    sensor.acquireData(1, 1)
    temp = sensor.readTemperature()
    humid = sensor.readHumidity()
    print(str(round(temp, 2)) + "C\t" + str(round(humid, 2)))
    sleep(0.5)
