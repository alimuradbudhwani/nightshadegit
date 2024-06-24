/**********************************************************
  ICM20948_9DoF_IMU - NightShade_Treo by NightShade Electronics

  This sketch demonstrates the functionality of the
  NightShade Trēo ICM20948 9 DoF IMU module. (NSE-1124-1) 
  It reads the accelerometer, gyro, and magnetometer data 
  from the sensor and prints it over Serial at 115200 
  baudrate.

  Created by Aaron D. Liebold
  on February 15, 2021

  Links:
  NightShade Trēo System: https://nightshade.net/treo
  Product Page: https://nightshade.net/product/treo-9-dof-imu-icm20948/

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
NightShade_Treo_ICM20948 sensor(1);

int main() {
	sensor.begin();


	while(1) {
		sensor.getData(1, 1);
		float accelX = (float) sensor.readAccelX() / sensor.accelSensitivity(); // G
		float accelY = (float) sensor.readAccelY() / sensor.accelSensitivity(); // G
		float accelZ = (float) sensor.readAccelZ() / sensor.accelSensitivity(); // G
		float gyroX = (float) sensor.readGyroX() / sensor.gyroSensitivity(); // dps
		float gyroY = (float) sensor.readGyroY() / sensor.gyroSensitivity(); // dps
		float gyroZ = (float) sensor.readGyroZ() / sensor.gyroSensitivity(); // dps
		float magX = (float) (sensor.readCompassX() * 15) / 100; // uT
		float magY = (float) (sensor.readCompassY() * 15) / 100; // uT
		float magZ = (float) (sensor.readCompassZ() * 15) / 100; // uT

		// Calculate Azimuth at X+ (2-axis)
		float azimuth =  atan2(-1*magY, magX) * 180 / M_PI; 
		if (azimuth < 0) azimuth += 360.0; // Keep azimuth positive (0 - 360deg)


		// Print Results
		printf("Accel X: %.2fg\tY: %.2fg\tZ: %.2fg \tGyro X: %.2fdps \tY: %.2fdps \tZ: %.2fdps \tCompass Heading: %.2fdeg\n", accelX, accelY, accelZ, gyroX, gyroY, gyroZ, azimuth);

		sleep(1);
	}
	return 0;
}
