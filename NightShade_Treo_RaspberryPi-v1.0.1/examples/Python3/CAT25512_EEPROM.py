"""
CAT25512_EEPROM - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo CAT25512 EEPROM module. (NSE-1138-1) It
performs write  and read cycles, playing "telephone" with
the EEPROM.

Created by Aaron D. Liebold
on Feburary 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page: https://nightshade.net/product/treo-512kb-eeprom-cat25512/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

import NightShade_Treo
from time import sleep

eeprom = NightShade_Treo.CAT25512(1, 7)

buffer =  "Only 128 bytes can be read or written to the CAT25512 EEPROM at a time. This is called a page write/read....................128B"

print("Initial character string: \"" + buffer + "\"")

eeprom.begin()

while True:
	print("\nWriting last string to memory...")
	eeprom.startMemoryWrite(0x00);
	for x in range(0,128,1):
		eeprom.write(ord(buffer[x]))
	eeprom.endMemoryWrite()

	print("Clearing software buffer.")
	buffer = ""
	print("Buffer cleared.")

	print("Reading memory.")
	eeprom.requestMemoryRead(0x00, 128)
	buffer = chr(eeprom.read())
	for i in range(1,128,1):
		buffer = buffer + chr(eeprom.read())
	print("String read from EEPROM:\"" + buffer + "\"")

	sleep(0.5)
