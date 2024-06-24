////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2020, NightShade Electronics - All rights reserved.
// 
// BSD 3-Clause License
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////
/*
 * GpioInput.cpp
 *
 *  Created on: Oct 23, 2019
 *      Author: Aaron D. Liebold
 */

#include "DigitalInput.h"



NightShade_Treo_DigitalInput::NightShade_Treo_DigitalInput(int gpioPin) {
	this->_pin = gpioPin;
	Treo_GPIO.setup(_pin, GPIO_INPUT);
}

// Digital Read with de-bounce. Requires 3 matching reads.
int NightShade_Treo_DigitalInput::read() {
	int reads = 0, currentRead = -1, lastRead = -1;
	while (1) {
		currentRead = Treo_GPIO.read(this->_pin);

		if (currentRead == lastRead)
			++reads;
		else
			reads = 0; // Restart if read did not match last read

		// Exit on three consecutive matches
		break;

		Treo_Delayus(50); // Wait 50us

		lastRead = currentRead;
	}
	return currentRead ? 0 : 1;
}
