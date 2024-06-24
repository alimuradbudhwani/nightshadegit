/**********************************************************
  HT16K33_7SegmentDisplay - NightShade_Treo by NightShade Electronics

  This sketch demonstrates the functionality of the  
  NightShade Trēo HT16K33 4 or 8 character 7-segment 
  display module. (NSE-1157-1/2) It prints the program time 
  to the display in seconds.
  
  Created by Aaron D. Liebold
  on February 15, 2021

  Links:
  NightShade Trēo System: https://nightshade.net/treo
  Product Page (8 Character): https://nightshade.net/product/treo-8-character-7-seg-display-ht16k33/
  Product Page (4 Character): https://nightshade.net/product/treo-4-character-7-seg-display-ht16k33/

  Distributed under the MIT license
  Copyright (C) 2021 NightShade Electronics
  https://opensource.org/licenses/MIT
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <unistd.h>

// Declare Objects
NightShade_Treo_HT16K33 display(1);

int main() {
	display.begin();

	unsigned long time = 0;

	while (1) {
		display.clearDigits();
		display.printNumber((float) time/100, 2);
		display.writeDisplay();
		usleep(10000);
		++time;
	}
	return 0;
}
