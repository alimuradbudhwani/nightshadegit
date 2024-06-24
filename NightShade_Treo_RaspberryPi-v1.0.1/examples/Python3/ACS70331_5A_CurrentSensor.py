"""
ACS70331_5A_CurrentSensor - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo ACS711 5A current sensing module.
(NSE-1130-1) It prints the current value passing
through the sensor to Serial at 115200 baudrate.

Created by Aaron D. Liebold
on February 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page: https://nightshade.net/product/treo-5a-current-sensor-acs70331/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

# Include NightShade Treo Library
import NightShade_Treo
from time import sleep

# Declare Objects
current = NightShade_Treo.ACS70331(1)

current.begin()

while True:
    value = current.read()
    print(str(value) + "mA")
    sleep(0.5)

