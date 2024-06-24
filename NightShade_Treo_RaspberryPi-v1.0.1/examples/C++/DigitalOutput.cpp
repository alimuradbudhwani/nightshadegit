/**********************************************************
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
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <unistd.h>

// Declare Objects (Dual Output Device - GPIO0: D5, GPIO1: D4)
NightShade_Treo_DigitalOutput out0(16);
NightShade_Treo_DigitalOutput out1(12);

int main() {
	while(1) {
		out0.on();
		usleep(500000);
		out0.off();
		usleep(500000);
		out1.on();
		usleep(500000);
		out1.off();
		usleep(500000);
	}
	return 0;
}
