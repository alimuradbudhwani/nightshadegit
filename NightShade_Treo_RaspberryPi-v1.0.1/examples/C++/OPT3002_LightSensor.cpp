/**********************************************************
  OPT3002_LightSensor - NightShade_Treo by NightShade Electronics

  This sketch demonstrates the functionality of the  
  NightShade Trēo OPT3002 light sensing module. 
  (NSE-1131-1) It prints the measured light level to 
  Serial at 115200 baudrate.
  
  Created by Aaron D. Liebold
  on February 15, 2021

  Links:
  NightShade Trēo System: https://nightshade.net/treo
  Product Page: https://nightshade.net/product/treo-light-sensor-opt3002/

  Distributed under the MIT license
  Copyright (C) 2021 NightShade Electronics
  https://opensource.org/licenses/MIT
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <stdio.h>
#include <unistd.h>

// Declare Objects
NightShade_Treo_OPT3002 sensor(1);

int main() {
	sensor.begin();

	while (1) {
		float lightValue = sensor.readLightLevel();
		printf("Light Level = %.1fnW/cm2\n", lightValue);
		sleep(1);
	}
	return 0;
}
