"""
DigitalOutput - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo digital output modules like LEDs and
relays.

Created by Aaron D. Liebold
on February 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

# Include NightShade Treo Library
import NightShade_Treo
from time import sleep

# Declare Objects (Dual Output Device - GPIO0: D5, GPIO1: D4)
out0 = NightShade_Treo.DigitalOutput(16)
out1 = NightShade_Treo.DigitalOutput(12)

while True:
    out0.on()
    sleep(0.5)
    out0.off()
    sleep(0.5)
    out1.on()
    sleep(0.5)
    out1.off()
    sleep(0.5)
