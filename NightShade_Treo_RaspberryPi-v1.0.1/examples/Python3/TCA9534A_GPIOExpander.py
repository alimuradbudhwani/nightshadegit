"""
TCA9534_GPIOExpander - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo TCA9534 GPIO expander module.
(NSE-1147-1) It prints the digital value being input to
each GPIO channel to Serial at 115200 baudrate.

Created by Aaron D. Liebold
on February 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page: https://nightshade.net/product/treo-gpio-expander-tca9534a/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

# Include NightShade Treo Library
import NightShade_Treo
from time import sleep

# Declare Objects
sensor = NightShade_Treo.TCA9534A(1)

sensor.begin()
sensor.setPortMode(0xFF)    # Set all pins to input

while True:
	# Read input state
	port = sensor.readPortInput()
	
	print(bin(port))
	sleep(1)
