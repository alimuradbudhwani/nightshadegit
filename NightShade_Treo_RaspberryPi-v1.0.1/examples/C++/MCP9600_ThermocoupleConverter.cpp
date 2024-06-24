/**********************************************************
  MCP9600_ThermocoupleConverter - NightShade_Treo by NightShade Electronics

  This sketch demonstrates the functionality of the  
  NightShade Trēo MCP9600 thermocouple converter module. 
  (NSE-1133-1) It prints the measured temperature to 
  Serial at 115200 baudrate.
  
  Created by Aaron D. Liebold
  on February 15, 2021

  Links:
  NightShade Trēo System: https://nightshade.net/treo
  Product Page: https://nightshade.net/product/treo-thermocouple-converter-mcp96l00/

  Distributed under the MIT license
  Copyright (C) 2021 NightShade Electronics
  https://opensource.org/licenses/MIT
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <stdio.h>
#include <unistd.h>

// Declare Objects
NightShade_Treo_MCP9600 temp(1);

int main() {
	temp.begin();

	while (1) {
		if (temp.dataReady()) {
			float T = (float) temp.readTemperature()*0.0625;
			printf("Temp: %.2f°C\n", T);
		}
		sleep(1);
	}
	return 0;
}
