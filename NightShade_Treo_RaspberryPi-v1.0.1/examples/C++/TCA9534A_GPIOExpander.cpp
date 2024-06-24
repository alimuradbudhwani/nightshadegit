/**********************************************************
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
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <stdio.h>
#include <unistd.h>
#include <bitset>

// Declare Objects
NightShade_Treo_TCA9534A sensor(1);

int main() {
	sensor.begin();

	sensor.setPortMode(0xFF); // Set all pins to input

	while(1) {
		// Read input state
		uint8_t input = sensor.readPortInput();

		// Print intput with leading zeros
		for (int x = 7; x >= 0; --x) {
			if ( (1 << x) & input ) {
				printf("1");
			} else {
				printf("0");
			}
		}
		printf("\n");
		sleep(1);
	}
	return 0;
}