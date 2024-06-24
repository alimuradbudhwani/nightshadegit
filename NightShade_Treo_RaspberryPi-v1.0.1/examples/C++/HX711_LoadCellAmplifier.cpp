/**********************************************************
  HX711_LoadCellAmplifier - NightShade_Treo by NightShade Electronics

  This sketch demonstrates the functionality of the  
  NightShade Trēo HX711 load cell amplifier module. 
  (NSE-1132-1) It prints the measured signal from channel A 
  to Serial at 115200 baudrate.
  
  Created by Aaron D. Liebold
  on February 15, 2021

  Links:
  NightShade Trēo System: https://nightshade.net/treo
  Product Page: https://nightshade.net/product/treo-24-bit-load-cell-adc-hx711/

  Distributed under the MIT license
  Copyright (C) 2021 NightShade Electronics
  https://opensource.org/licenses/MIT
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <stdio.h>
#include <unistd.h>

// Declare Objects
NightShade_Treo_HX711 sensor(16, 12);

int main() {
	sensor.begin();

	// Tare sensor (5 reads)
	unsigned long tare = 0;
	for (int x=0; x<5; ++x) tare += sensor.conversion(0);
	sensor.setOffset(tare / 5);

	while(1) {
		printf("%i\n", sensor.conversion(0));
		sleep(1);
	}
	return 0;
}