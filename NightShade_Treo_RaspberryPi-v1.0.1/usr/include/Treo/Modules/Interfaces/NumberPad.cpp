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
 * NumberPad.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: Aaron D. Liebold
 */

#include "NumberPad.h"

NightShade_Treo_NumberPad::NightShade_Treo_NumberPad(int port,
		uint8_t slaveAddress, uint32_t clockSpeed) {
	this->_port = port;
	this->_slaveAddress = slaveAddress;
	this->_clockSpeed = clockSpeed;
	this->_connectorFlipped = 0;
	this->gpio = new NightShade_Treo_TCA9534A(this->_port, this->_slaveAddress,
			this->_clockSpeed);
}

NightShade_Treo_NumberPad::NightShade_Treo_NumberPad(int port) {
	this->_port = port;
	this->_slaveAddress = TCA9534A_DEFAULT_ADDR;
	this->_clockSpeed = TCA9534A_DEFAULT_CLOCKSPEED;
	this->_connectorFlipped = 0;
	this->gpio = new NightShade_Treo_TCA9534A(this->_port, this->_slaveAddress,
			this->_clockSpeed);
}

NightShade_Treo_NumberPad::~NightShade_Treo_NumberPad() {
	delete this->gpio;
}

void NightShade_Treo_NumberPad::begin() {
	this->gpio->begin();
	this->gpio->setPortMode(0xF0);
	this->gpio->setPortOutput(0x00);
}

int NightShade_Treo_NumberPad::read() {
	int row = 0, column = 0;
	uint8_t read;

	// Read column
	this->gpio->setPortMode(0xF0);
	this->gpio->setPortOutput(0x00);
	Treo_Delayus(50); // Settling time
	read = this->gpio->readPortInput();
	read = ~read & 0xF0; // Isolate columns
	for (uint8_t x = 4; x < 8; ++x) {
		if ((read >> x) & 0x01) {
			column = !column ? x : -1;
		}
	}

	// Check if button is pressed
	if (!read) {
		column = -1;
		row = -1;
	}

	// Read row
	this->gpio->setPortMode(0x0F);
	this->gpio->setPortOutput(0x00);
	Treo_Delayus(50); // Settling time
	read = this->gpio->readPortInput();
	read = ~read & 0x0F; // Isolate rows
	for (uint8_t x = 0; x < 4; ++x) {
		if ((read >> x) & 0x01)
			row = !row ? x : -1;
	}

// Reset state
	this->gpio->setPortMode(0xF0);
	this->gpio->setPortOutput(0x00);

	if (row >= 0 && column >= 0) {
		if (this->_connectorFlipped) {
			return this->_numbers[row][column - 4];
		} else {
			return this->_numbers[7 - column][3 - row];
		}
	} else {
		return -1; // No key pressed
	}
}

int NightShade_Treo_NumberPad::buttonPressed() {
// Set pull downs
	this->gpio->setPortMode(0xF0);
	this->gpio->setPortOutput(0x00);
	Treo_Delayus(50); // Settling time

// Read columns
	uint8_t read = this->gpio->readPortInput();
	read = ~read & 0xF0; // Isolate columns

	return read ? 1 : 0;
}

void NightShade_Treo_NumberPad::connectorFlipped(int isFlipped) {
	this->_connectorFlipped = isFlipped ? 1 : 0;
}

