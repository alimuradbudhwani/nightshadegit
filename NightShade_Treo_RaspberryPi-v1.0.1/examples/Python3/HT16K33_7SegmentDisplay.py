"""
HT16K33_7SegmentDisplay - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo HT16K33 4 or 8 character 7-segment
display module. (NSE-1157-1/2) It prints the program time
to the display in seconds.

Created by Aaron D. Liebold
on February 15, 2021

Links:
NightShade Trēo System: https://nightshade.net/treo
Product Page (8 Character): https://nightshade.net/product/treo-8-character-7-seg-display-ht16k33/
Product Page (4 Character): https://nightshade.net/product/treo-4-character-7-seg-display-ht16k33/

Distributed under the MIT license
Copyright (C) 2021 NightShade Electronics
https://opensource.org/licenses/MIT
"""

# Include NightShade Treo Library
import NightShade_Treo
from time import time

# Declare Objects
display = NightShade_Treo.HT16K33(1)
display.begin()

startTime = time()

while True:
    display.clearDigits()
    display.printNumber(time() - startTime, 2)
    display.writeDisplay()
