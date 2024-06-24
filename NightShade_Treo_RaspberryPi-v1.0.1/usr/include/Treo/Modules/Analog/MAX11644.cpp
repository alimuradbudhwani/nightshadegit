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
 * MAX11644.cpp
 *
 *  Created on: Oct 24, 2019
 *      Author: Aaron D. Liebold
 */

#include "MAX11644.h"


#define TWI_START Treo_TWI.begin(this->_port, this->_slaveAddress, this->_clockSpeed)
#define TWI_STOP Treo_TWI.end()

NightShade_Treo_MAX11644::NightShade_Treo_MAX11644(int port,
		uint8_t slaveAddress, uint32_t clockSpeed) {
	this->_port = port;
	this->_clockSpeed = clockSpeed;
	this->_slaveAddress = slaveAddress;
}

NightShade_Treo_MAX11644::NightShade_Treo_MAX11644(int port) {
	this->_port = port;
	this->_clockSpeed = MAX11644_DEFAULT_CLOCKSPEED;
	this->_slaveAddress = MAX11644_DEFAULT_SLAVEADDR;
}

int NightShade_Treo_MAX11644::begin() {
	// Write Setup Reg
	uint8_t buffer[1];
	buffer[0] = {0xD2};
	TWI_START;
	int ret = Treo_TWI.write(buffer, 1, 1);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MAX11644::acquireAllChannels() {
	uint8_t buffer[4];
	buffer[0] = 0x03;

	TWI_START;
	Treo_TWI.write(buffer, 1, 1);
	int ret = Treo_TWI.read(buffer, 4, 1);
	TWI_STOP;

	this->_adcValues[0] = (uint16_t) (buffer[0] << 8) | buffer[1];
	this->_adcValues[1] = (uint16_t) (buffer[2] << 8) | buffer[3];
	return ret;
}

uint16_t NightShade_Treo_MAX11644::retrieveChannelData(int channel) {
	return this->_adcValues[channel];
}

uint16_t NightShade_Treo_MAX11644::acquireSingleChannel(int channel) {
	uint8_t buffer[2];

	// Config Reg
	buffer[0] = channel ? 0x63 : 0x61;

	TWI_START;
	Treo_TWI.write(buffer, 1, 1);
	int ret = Treo_TWI.read(buffer, 2, 1);
	TWI_STOP;

	if (ret == 0) return (uint16_t) ((buffer[0] & 0x0F) << 8) | buffer[1];
	else return -1;
}

int16_t NightShade_Treo_MAX11644::acquireDifferentialChannel(
		int positveChannel) {
	uint8_t buffer[2];

	TWI_START;

	// Setup & Config Reg
	buffer[0] = 0xD6;
	buffer[1] = positveChannel ? 62 : 60;
	Treo_TWI.write(buffer, 2, 1);

	// Read ADC
	Treo_TWI.read(buffer, 2, 1);

	// Restore Setup
	buffer[0] = 0xD2;
	Treo_TWI.write(buffer, 1, 1);

	TWI_STOP;

	if (!(buffer[0] & (1 << 3)))
		buffer[0] &= 0x0F; // Set as negative or positive

	return (int16_t) ((0x0F & buffer[0]) << 8) | buffer[1];
}

int16_t NightShade_Treo_MAX11644::acquireDifferentialChannel() {
	return this->acquireDifferentialChannel(0);
}
