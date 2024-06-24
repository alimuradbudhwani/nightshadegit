  /**********************************************************
  ADXL345_Accelerometer - NightShade_Treo by NightShade Electronics
  
  This sketch demonstrates the functionality of the 
  NightShade Trēo ADXL345 accelerometer module. (NSE-1123-1) It it reads the 
  accelerations measured by the sensor and prints them over 
  serial at 115200 baudrate.
  
  Created by Aaron D. Liebold
  on Februsary 15, 2021

  Links:
  NightShade Trēo System: https://nightshade.net/treo
  Product Page: https://nightshade.net/product/treo-3-axis-accelerometer-adxl345/

  Distributed under the MIT license
  Copyright (C) 2021 NightShade Electronics
  https://opensource.org/licenses/MIT
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <stdio.h>
#include <unistd.h>

// Declare Objects
NightShade_Treo_ADXL345 accel(1);

int main() {
	accel.begin();
	accel.setMeasurementRange(0);

	while(1) {
		accel.retrieveData();
		int X = accel.readX();
		int Y = accel.readY();
		int Z = accel.readZ();
		printf("X: %i\tY: %i\tZ: %i\n", X, Y, Z);
		sleep(1);
	}
	return 0;
}