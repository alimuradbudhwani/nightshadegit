"""
MAX11612_ADC - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo MAX11612 ADC module. (NSE-1127-1/2) It
prints the voltage present on channel 0 to Serial at
115200 baudrate. 10x mode can be enabled for the
NSE-1127-2.

Created by Aaron D. Liebold
on Feburary 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page: https://nightshade.net/product/treo-4-2-channel-12-bit-adc-max11612/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

# Include NightShade Treo Library
import NightShade_Treo
from time import sleep

# Declare Objects
adc = NightShade_Treo.MAX11612(1)

# Set to 1 to enable the 10x input for NSE-1127-2
ENABLE_10X_INPUT = 0

adc.begin()

while True:
	adc.acquireAllChannels()
	if ENABLE_10X_INPUT:
		print(str(10 * adc.readChannel(0)) + "mV")
	else:
		print(str(adc.readChannel(0)) + "mV")
	
	sleep(1)
