/**********************************************************
  ACS70331_5A_CurrentSensor - NightShade_Treo by NightShade Electronics

  This sketch demonstrates the functionality of the  
  NightShade Trēo ACS711 5A current sensing module. 
  (NSE-1130-1) It prints the current value passing 
  through the sensor to Serial at 115200 baudrate.
  
  Created by Aaron D. Liebold
  on February 15, 2021

  Links:
  NightShade Trēo System: https://nightshade.net/treo
  Product Page: https://nightshade.net/product/treo-5a-current-sensor-acs70331/

  Distributed under the MIT license
  Copyright (C) 2021 NightShade Electronics
  https://opensource.org/licenses/MIT
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <stdio.h>
#include <unistd.h>

// Declare Objects
NightShade_Treo_ACS70331 current(1);

int main() {
	current.begin();

	while (1) {
		int value = current.read();
		printf("%imA\n", value);
		sleep(1);
	}
	return 0;
}
