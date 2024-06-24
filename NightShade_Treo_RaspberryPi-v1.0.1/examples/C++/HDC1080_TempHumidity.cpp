/**********************************************************
  HDC1080_TempHumidity - NightShade_Treo by NightShade Electronics

  This sketch demonstrates the functionality of the  
  NightShade Trēo HDC1080 temperature and humidity sensing 
  module. (NSE-1134-1) It prints the temperature and 
  humidity values to Serial at 115200 baudrate.
  
  Created by Aaron D. Liebold
  on February 15, 2021

  Links:
  NightShade Trēo System: https://nightshade.net/treo
  Product Page: https://nightshade.net/product/treo-temp-humidity-sensor-hdc1080/

  Distributed under the MIT license
  Copyright (C) 2021 NightShade Electronics
  https://opensource.org/licenses/MIT
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <stdio.h>
#include <unistd.h>

NightShade_Treo_HDC1080 sensor(1);

unsigned long lastHeater = 0, lastUpdate = 0;

int main() {
	sensor.begin();
	sensor.setTemperatureResolution(0);
	sensor.setHumidityResolution(0);
	
	unsigned int deviceID = sensor.readDeviceId();
	unsigned int mfgID = sensor.readManufacturerId();
	unsigned long serialID = sensor.readSerialId();
	printf("Device ID: 0x%X\nManufacturer ID: 0x%X\nSerial ID: 0x%X\n", deviceID, mfgID, serialID);

	usleep(1000);
	while (1) {
		sensor.acquireData(1, 1);
		float temp = sensor.readTemperature();
		float humid = sensor.readHumidity();
		printf("%.2fC\t%.2f\n", (double) temp, (double) humid);
		sleep(1);
	}
	return 0;
}
