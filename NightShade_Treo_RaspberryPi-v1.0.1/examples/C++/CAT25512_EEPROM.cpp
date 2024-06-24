/**********************************************************
  CAT25512_EEPROM - NightShade_Treo by NightShade Electronics

  This sketch demonstrates the functionality of the
  NightShade Trēo CAT25512 EEPROM module. (NSE-1138-1) It 
  performs write  and read cycles, playing "telephone" with 
  the EEPROM.

  Created by Aaron D. Liebold
  on February 15, 2021

  Links:
  NightShade Trēo System: https://nightshade.net/treo
  Product Page: https://nightshade.net/product/treo-512kb-eeprom-cat25512/

  Distributed under the MIT license
  Copyright (C) 2021 NightShade Electronics
  https://opensource.org/licenses/MIT
**********************************************************/

#include <Treo/NightShade_Treo.h>
#include <stdio.h>
#include <unistd.h>

NightShade_Treo_CAT25512 eeprom(1, 7);

// Buffer must have an extra byte for the null character ('\0') at the end of the string 
char buffer[129] =  "Only 128 bytes can be read or written to the CAT25512 EEPROM at a time. This is called a page write/read....................128B";

int main() {
	printf("Initial character string: \"%s\"\n", buffer);
	
	eeprom.begin();
	eeprom.setProtectionLevel(0);
	
	while(1) {
		printf("\nWriting last string to memory...\n");
		
		eeprom.startMemoryWrite(0x00);
		for (int x = 0; x < 128; ++x) eeprom.write(buffer[x]);
		eeprom.endMemoryWrite();
		
		printf("Clearing software buffer.\n");
		for (int x = 0; x < 128; ++x) buffer[x] = '.';
		printf("Buffer cleared.\n");
		printf("Reading memory.\n");
		eeprom.requestMemoryRead(0x00, 128);
		for (int x = 0; x < 128; ++x) buffer[x] = eeprom.read();
		printf("String read from EEPROM:\"%s\"\n", buffer);

		sleep(1);
	}
	return 0;
}
