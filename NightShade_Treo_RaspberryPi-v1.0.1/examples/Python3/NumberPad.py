"""
NumberPad - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo Number Padmodule. (NSE-1142-1) It prints
each pressed character to Serial at 115200 baudrate.

Created by Aaron D. Liebold
on February 17, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page: https://nightshade.net/product/treo-number-pad-tca9534a/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

# Include NightShade Treo Library
import NightShade_Treo
from time import sleep

# Declare Objects
keypad = NightShade_Treo.NumberPad(1)

keypad.begin()

while True:
	if keypad.buttonPressed():
		key = keypad.read()
		if key > 0:
			print(chr(key))
		while (keypad.buttonPressed()):
			pass
