"""
DigitalInput - NightShade_Treo by NightShade Electronics

This sketch demonstrates the functionality of the
NightShade Trēo digital input modules like buttons and
switches. It prints any chance in state to Serial at
115200 baudrate.

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

# Declare Objects (Dual Input Device - GPIO0: D5, GPIO1: D4)
input0 = NightShade_Treo.DigitalInput(16)
input1 = NightShade_Treo.DigitalInput(12)

while True:
    if input0.read():
        print("Input0 Activated!")
        while input0.read(): # Wait until input is released
            pass
        print("Input0 Deactivated!")

    if input1.read():
        print("Input1 Activated!")
        while input1.read(): # Wait until input is released
            pass
        print("Input1 Deactivated!")

