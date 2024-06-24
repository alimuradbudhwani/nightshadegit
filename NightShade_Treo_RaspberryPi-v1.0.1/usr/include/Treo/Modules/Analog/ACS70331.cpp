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
 * ACS70331.cpp
 *
 *  Created on: Oct 24, 2019
 *      Author: Aaron D. Liebold
 */

#include "ACS70331.h"

NightShade_Treo_ACS70331::NightShade_Treo_ACS70331(int port,
		uint8_t slaveAddress, uint32_t clockSpeed) {
	this->_port = port;
	this->_clockSpeed = clockSpeed;
	this->_slaveAddress = slaveAddress;
	this->_offset = 0;
	this->_adc = new NightShade_Treo_MAX11644(this->_port, this->_slaveAddress,
			this->_clockSpeed);
}

NightShade_Treo_ACS70331::NightShade_Treo_ACS70331(int port) {
	this->_port = port;
	this->_clockSpeed = MAX11644_DEFAULT_CLOCKSPEED;
	this->_slaveAddress = MAX11644_DEFAULT_SLAVEADDR;
	this->_offset = 0;
	this->_adc = new NightShade_Treo_MAX11644(this->_port, this->_slaveAddress,
			this->_clockSpeed);
}

NightShade_Treo_ACS70331::~NightShade_Treo_ACS70331() {
	delete this->_adc; // Delete MAX11644 instance
}

int NightShade_Treo_ACS70331::begin() {
	return this->_adc->begin();
}

int NightShade_Treo_ACS70331::read() {
	int measurement = this->_adc->acquireSingleChannel(0);
	measurement -= 1500; // Subtract zero-current voltage
	measurement *= 5; // Convert to mA
	return measurement - this->_offset; // Subtract offset current
}

void NightShade_Treo_ACS70331::setOffset(int offset) {
	this->_offset = offset;
}
