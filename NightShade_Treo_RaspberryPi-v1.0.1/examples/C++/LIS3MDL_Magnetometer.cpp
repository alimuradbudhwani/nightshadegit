/**********************************************************
  LIS3MDL_Magnetometer - NightShade_Treo by NightShade Electronics

  This sketch demonstrates the functionality of the
  NightShade Trēo LIS3MDL magnetometer module. (NSE-1126-1) 
  It prints the magnetometer data, a calculated heading, 
  and temperature from the sensor and prints in out as 
  Serial at 115200 baudrate.

  Created by Aaron D. Liebold
  on February 15, 2021

  Links:
  NightShade Trēo System: https://nightshade.net/treo
  Product Page: https://nightshade.net/product/treo-3-axis-magnetometer-lis3mdl/

  Distributed under the MIT license
  Copyright (C) 2021 NightShade Electronics
  https://opensource.org/licenses/MIT
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

// Declare Objects
NightShade_Treo_LIS3MDL sensor(1);

int main() {
	sensor.begin();

	while(1) {
		sensor.acquireMagData();
		float X = (float) sensor.readX() / sensor.oneGaussValue(); // X value in Gauss
		float Y = (float) sensor.readY() / sensor.oneGaussValue(); // Y value in Gauss
		float Z = (float) sensor.readZ() / sensor.oneGaussValue(); // Z value in Gauss
		float temp = (float) sensor.readTemp() / 10; // Retrieve temperature in deg C

		// Calculate Azimuth at X+ (2-axis)
		float azimuth =  atan2(Y, X) * 180 / M_PI;
		if (azimuth < 0) azimuth += 360.0; // Keep azimuth positive (0 - 360deg)

		printf("Az: %.2fdeg \t%.2fG\t%.2fG\t%.2fG\t%.2fC\t\n", (double) azimuth, (double) X, (double) Y, (double) Z, (double) temp);

		sleep(1);
	}
	return 0;
}
