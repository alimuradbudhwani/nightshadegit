/**********************************************************
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
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <stdio.h>

// Declare Objects (Dual Input Device - GPIO0: D5, GPIO1: D4)
NightShade_Treo_DigitalInput input0(16);
NightShade_Treo_DigitalInput input1(12);

int main() {
	while(1) {
		if (input0.read()) {
			printf("Input0 Activated!\n");
			while (input0.read()); // Wait until input is released
			printf("Input0 Deactivated!\n");
		}
		if (input1.read()) {
			printf("Input1 Activated!\n");
			while (input1.read()); // Wait until input is released
			printf("Input1 Deactivated!\n");
		}
	}
	return 0;
}
