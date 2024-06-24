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
 * TCA9534A.cpp
 *
 *  Created on: Nov 8, 2019
 *      Author: Aaron D. Liebold
 */

#include "TCA9534A.h"


#define TWI_START Treo_TWI.begin(this->_port, this->_slaveAddress, this->_clockSpeed)
#define TWI_STOP Treo_TWI.end()

NightShade_Treo_TCA9534A::NightShade_Treo_TCA9534A(int port,
		uint8_t slaveAddress, uint32_t clockSpeed) {
	this->_port = port;
	this->_slaveAddress = slaveAddress;
	this->_clockSpeed = clockSpeed;
}

NightShade_Treo_TCA9534A::NightShade_Treo_TCA9534A(int port) {
	this->_port = port;
	this->_slaveAddress = TCA9534A_DEFAULT_ADDR;
	this->_clockSpeed = TCA9534A_DEFAULT_CLOCKSPEED;
}

int NightShade_Treo_TCA9534A::begin() {
	return 0;
}

int NightShade_Treo_TCA9534A::setPortMode(uint8_t portMode) {
	TWI_START;
	int ret = Treo_TWI.writeByte(REGCON_CONFIG, portMode);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_TCA9534A::setPortOutput(uint8_t portOutput) {
	TWI_START;
	int ret = Treo_TWI.writeByte(REGCON_OUTPUT, portOutput);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_TCA9534A::setPortPolarity(uint8_t portPolarityInversion) {
	TWI_START;
	int ret = Treo_TWI.writeByte(REGCON_POLINV, portPolarityInversion);
	TWI_STOP;
	return ret;
}

uint8_t NightShade_Treo_TCA9534A::readPortMode() {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REGCON_CONFIG);
	TWI_STOP;
	return reg;
}

uint8_t NightShade_Treo_TCA9534A::readPortOutput() {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REGCON_OUTPUT);
	TWI_STOP;
	return reg;
}

uint8_t NightShade_Treo_TCA9534A::readPortInput() {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REGCON_INPUT);
	TWI_STOP;
	return reg;
}

uint8_t NightShade_Treo_TCA9534A::readPortPolarity() {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REGCON_POLINV);
	TWI_STOP;
	return reg;
}


int NightShade_Treo_TCA9534A::setPinMode(int pin, int mode) {
	uint8_t reg = this->readPortMode();
	reg &= ~(1 << pin);
	if (mode) reg |= (1 << pin);
	return this->setPortMode(reg);
}

int NightShade_Treo_TCA9534A::setPinOutput(int pin, int output) {
	uint8_t reg = this->readPortOutput();
	reg &= ~(1 << pin);
	if (output) reg |= (1 << pin);
	return this->setPortOutput(reg);
}

int NightShade_Treo_TCA9534A::setPinPoarity(int pin, int polarityInverted) {
	uint8_t reg = this->readPortPolarity();
	reg &= ~(1 << pin);
	if (polarityInverted) reg |= (1 << pin);
	return this->setPortPolarity(reg);
}

int NightShade_Treo_TCA9534A::readPinMode(int pin) {
	uint8_t reg = readPortMode();
	return (reg >> pin) & 0x01;
}

int NightShade_Treo_TCA9534A::readPinOutput(int pin) {
	uint8_t reg = readPortOutput();
	return (reg >> pin) & 0x01;
}

int NightShade_Treo_TCA9534A::readPinInput(int pin) {
	uint8_t reg = readPortInput();
	return (reg >> pin) & 0x01;
}

int NightShade_Treo_TCA9534A::readPinPolarity(int pin) {
	uint8_t reg = readPortPolarity();
	return (reg >> pin) & 0x01;
}

