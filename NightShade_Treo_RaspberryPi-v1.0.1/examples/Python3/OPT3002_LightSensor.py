"""
OPT3002_LightSensor - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo OPT3002 light sensing module.
(NSE-1131-1) It prints the measured light level to
Serial at 115200 baudrate.

Created by Aaron D. Liebold
on February 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page: https://nightshade.net/product/treo-light-sensor-opt3002/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

# Include NightShade Treo Library
import NightShade_Treo
from time import sleep

# Declare Objects
sensor = NightShade_Treo.OPT3002(1)

sensor.begin()

while True:
  lightValue = sensor.readLightLevel()
  print("Light Level = " + str(round(lightValue, 1)) + "nW/cm2")
  sleep(0.5)
