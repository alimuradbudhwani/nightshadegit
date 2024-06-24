/**********************************************************
  NumberPad - NightShade_Treo by NightShade Electronics

  This sketch demonstrates the functionality of the  
  NightShade Trēo Number Padmodule. (NSE-1142-1) It prints 
  each pressed character to Serial at 115200 baudrate.
  
  Created by Aaron D. Liebold
  on February 17, 2021

  Links:
  NightShade Trēo System: https://nightshade.net/treo
  Product Page: https://nightshade.net/product/treo-number-pad-tca9534a/

  Distributed under the MIT license
  Copyright (C) 2021 NightShade Electronics
  https://opensource.org/licenses/MIT
**********************************************************/

// Include NightShade Treo Library
#include <Treo/NightShade_Treo.h>
#include <stdio.h>
#include <unistd.h>

// Declare Objects
NightShade_Treo_NumberPad keypad(1);

int main() {
	keypad.begin();


	while (1) {
		if (keypad.buttonPressed()) {
			char key = keypad.read();
			if (key > 0) printf("%c\n", key);
			while (keypad.buttonPressed());
		}
	}
	return 0;
}