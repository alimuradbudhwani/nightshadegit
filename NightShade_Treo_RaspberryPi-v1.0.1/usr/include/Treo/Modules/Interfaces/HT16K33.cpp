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
 * HT16K33.cpp
 *
 *  Created on: May 28, 2020
 *      Author: Aaron D. Liebold
 */

#include "HT16K33.h"


#define TWI_START	Treo_TWI.begin(this->_port, this->_slaveAddress, this->_clockSpeed)
#define TWI_STOP	Treo_TWI.end()

NightShade_Treo_HT16K33::NightShade_Treo_HT16K33(int port, uint8_t slaveAddress,
		uint32_t clockSpeed) {
	this->_port = port;
	this->_slaveAddress = slaveAddress;
	this->_clockSpeed = clockSpeed;
	clearDigits();
}

NightShade_Treo_HT16K33::NightShade_Treo_HT16K33(int port) {
	this->_port = port;
	this->_slaveAddress = HT16K33_DEFAULT_SLAVEADDR;
	this->_clockSpeed = HT16K33_DEFAULT_CLOCKSPEED;
	clearDigits();
}

int NightShade_Treo_HT16K33::begin() {
	int error = 0;
	error += this->sleep(0);
	error += this->setOnBlinking(1, 0);
	error += this->setBrightness(15);
	error += this->setIntPin(0, 0);

	return error;
}

int NightShade_Treo_HT16K33::sleep(int enable) {
	uint8_t buffer = (enable ? HT16K33_REG_SYS_SLP : HT16K33_REG_SYS_WAKE);
	TWI_START;
	int ret = Treo_TWI.write(&buffer, 1, 1);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_HT16K33::setIntPin(int enableInt, int activeHigh) {
	uint8_t buffer = HT16K33_REG_INT_SETUP | (enableInt ? (1 << 0) : 0)
			| (activeHigh ? (1 << 1) : 0);
	TWI_START;
	int ret = Treo_TWI.write(&buffer, 1, 1);
	TWI_STOP;
	return ret;
}

void NightShade_Treo_HT16K33::setDigit(int digit, int8_t value) {
	if (value > 0) {
		this->_display[digit] = this->_characters[value % 16];
		if (value / 16) this->_display[digit] |= CHARACTER_PLUS_DECIMAL;
	} else this->_display[digit] = CHARACTER_NULL;
}

void NightShade_Treo_HT16K33::setDecimal(int digit, int enable) {
	if (enable) this->_display[digit] |= CHARACTER_PLUS_DECIMAL;
	else this->_display[digit] &= ~(CHARACTER_PLUS_DECIMAL);
}

void NightShade_Treo_HT16K33::printNumber(double value, int decimalPlaces,
		int offset) {
	int digit = 0, decimal = 0;

	// Create decimal places
	if (decimalPlaces >= 0) {
		for (int x = 0; x < decimalPlaces; ++x) {
			value *= 10;
			++decimal;
		}
	} else {
		while (value != (uint32_t) value) {
			value *= 10;
			++decimal;
		}
	}

	// Capture integer
	int32_t number = (int32_t) value;

	// Write digits
	digit = offset;
	do {
		if (digit >= 0) this->_display[digit] = _characters[number % 10];
		++digit;
		number /= 10;
	} while (number > 0 && digit < 16);

	// Write decimal place
	if (decimal > 0) this->setDecimal(decimal + offset, 1);
}

void NightShade_Treo_HT16K33::printNumber(double value, int decimalPlaces) {
	this->printNumber(value, decimalPlaces, 0);
}

void NightShade_Treo_HT16K33::printNumber(double value) {
	if (value == (int) value) this->printNumber(value, 0, 0);
	else this->printNumber(value, -1, 0);
}

void NightShade_Treo_HT16K33::clearDigits() {
	for (int x = 0; x < 16; ++x)
		this->_display[x] = 0;
}

int NightShade_Treo_HT16K33::writeDisplay() {
	uint8_t buffer[17];
	buffer[0] = HT16K33_REG_DATA_PTR;
	for (int x = 0; x < 16; x += 2) {
		buffer[x + 1] = this->_display[x / 2];
		buffer[x + 2] = 0x00;
	}
	TWI_START;
	int ret = Treo_TWI.write(buffer, 17, 1);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_HT16K33::setBrightness(uint8_t brightness) {
	int ret = -1;
	if (brightness >= 0 && brightness < 16) {
		uint8_t buffer = HT16K33_REG_DIM_SET | (brightness & 0x0F);
		TWI_START;
		ret = Treo_TWI.write(&buffer, 1, 1);
		TWI_STOP;
	}
	return ret;
}

int NightShade_Treo_HT16K33::setOnBlinking(int screenEnable,
		int blinkingValue) {
	uint8_t buffer = HT16K33_REG_DISP_SETUP | ((blinkingValue & 0x03) << 1)
			| (screenEnable ? 1 : 0);
	TWI_START;
	int ret = Treo_TWI.write(&buffer, 1, 1);
	TWI_STOP;
	return ret;
}
