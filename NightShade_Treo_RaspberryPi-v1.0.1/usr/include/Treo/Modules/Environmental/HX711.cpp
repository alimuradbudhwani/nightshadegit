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
 * HX711.cpp
 *
 *  Created on: Mar 4, 2020
 *      Author: Aaron D. Liebold
 */

#include "HX711.h"



NightShade_Treo_HX711::NightShade_Treo_HX711(int PDSCK_gpio0, int DOUT_gpio1) {
	_pdsckPin = PDSCK_gpio0;
	_doutPin = DOUT_gpio1;
	_sleeping = 0;
	_offset = 0;
}

int NightShade_Treo_HX711::begin() {
	Treo_GPIO.setup(_doutPin, GPIO_INPUT);
	Treo_GPIO.setup(_pdsckPin, GPIO_OUTPUT);
	Treo_GPIO.write(_pdsckPin, GPIO_LOW);
	return 0;
}

int32_t NightShade_Treo_HX711::conversion(int mode) {
	int32_t result = 0;
	uint8_t data[3] = { 0 };
	int nextMeasurement;

	if (!_sleeping) {
		switch (mode) {
		case 2:
			nextMeasurement = CHANNELB_32;
			break;
		case 1:
			nextMeasurement = CHANNELA_128;
			break;
		case 0:
		default:
			nextMeasurement = CHANNELA_64;
			break;
		}

		// Retrieve last conversion and set next conversion
		while (Treo_GPIO.read(_doutPin) == GPIO_HIGH)
			; // Wait for conversion
		Treo_Delayus(1);
		for (int x = 0; x < nextMeasurement; ++x) {
			int ones = 0;
			Treo_GPIO.write(_pdsckPin, GPIO_HIGH);
			Treo_Delayus(1);
			if (x < 24) {
				// Triple Sample
				if (Treo_GPIO.read(_doutPin) == GPIO_HIGH)
					++ones;
				Treo_Delayus(1);
				if (Treo_GPIO.read(_doutPin) == GPIO_HIGH)
					++ones;
				Treo_Delayus(1);
				if (Treo_GPIO.read(_doutPin) == GPIO_HIGH)
					++ones;
				data[2 - x / 8] += (ones > 1 ? 1 : 0) << (7 - (x % 8));
			} else {
				Treo_Delayus(3);
			}
			Treo_GPIO.write(_pdsckPin, GPIO_LOW);
			Treo_Delayus(5);
		}

		uint8_t dummyByte;
		if (data[2] & 0x80) {
			dummyByte = 0xFF;
		} else {
			dummyByte = 0x00;
		}

		result = (int32_t) ((uint32_t) dummyByte << 24)
				| ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8)
				| ((uint32_t) data[0]);

	} else {
		result = -1;
	}

	return result - _offset;
}

int  NightShade_Treo_HX711::enableSleep(int enable) {
	if (enable) {
		Treo_GPIO.write(_pdsckPin, GPIO_HIGH);
		_sleeping = 1;
	} else {
		Treo_GPIO.write(_pdsckPin, GPIO_LOW);
		_sleeping = 0;
	}
	return 0;
}

void NightShade_Treo_HX711::setOffset(int offset) {
	_offset = offset;
}

int NightShade_Treo_HX711::readOffset() {
	return _offset;
}
