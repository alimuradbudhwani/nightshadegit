"""
HX711_LoadCellAmplifier - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo HX711 load cell amplifier module.
(NSE-1132-1) It prints the measured signal from channel A
to Serial at 115200 baudrate.

Created by Aaron D. Liebold
on February 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page: https://nightshade.net/product/treo-24-bit-load-cell-adc-hx711/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

# Include NightShade Treo Library
import NightShade_Treo
from time import sleep

# Declare Objects
sensor = NightShade_Treo.HX711(16, 12)

sensor.begin()

# Tare sensor (5 reads)
tare = 0
for x in range(1,5,1):
    tare += sensor.conversion(0)
sensor.setOffset(int(tare / 5))


while True:
    value = sensor.conversion(0)
    print(str(value))
    sleep(1)
