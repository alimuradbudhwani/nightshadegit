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
 *  MAX11612.cpp
 *
 *  Created on: July 16, 2019
 *      Author: Aaron D. Liebold
 */

#include "MAX11612.h"

#define TWI_START	Treo_TWI.begin(this->_port, MAX11612_SLAVE_ADDR, this->_clockSpeed)
#define TWI_STOP	Treo_TWI.end()

NightShade_Treo_MAX11612::NightShade_Treo_MAX11612(int port,
		uint32_t clockSpeed) {
	_port = port;
	_clockSpeed = clockSpeed;
}

NightShade_Treo_MAX11612::NightShade_Treo_MAX11612(int port) {
	_port = port;
	_clockSpeed = MAX11612_DEFAULT_CLOCKSPEED;
}

int NightShade_Treo_MAX11612::begin() {
	_setup = SETUP_BASE | SETUP_INT_REF | SETUP_INTREF_ON | SETUP_EXT_CLOCK
			| SETUP_NO_RESET;
	_config = CONFIG_BASE | CONFIG_CONVT_ALL | CONFIG_SELECT_CH3
			| CONFIG_MODE_SNGLEND;
	uint8_t buffer[2] = { _setup, _config };
	TWI_START;
	int ret = Treo_TWI.write(buffer, 2, 1);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MAX11612::acquireAllChannels() {
	uint8_t data[8] = { 0 };
	TWI_START;
	int ret = Treo_TWI.read(data, 8, 1);
	TWI_STOP;
	_adcValues[0] = (uint16_t) (data[0] & 0x0F) << 8;
	_adcValues[0] |= data[1];
	_adcValues[1] = (uint16_t) (data[2] & 0x0F) << 8;
	_adcValues[1] |= data[3];
	_adcValues[2] = (uint16_t) (data[4] & 0x0F) << 8;
	_adcValues[2] |= data[5];
	_adcValues[3] = (uint16_t) (data[6] & 0x0F) << 8;
	_adcValues[3] |= data[7];
	return ret;
}

int NightShade_Treo_MAX11612::readChannel(int channel) {
	return _adcValues[channel];
}

int NightShade_Treo_MAX11612::readDifferentialChannel(int posCh) {
	uint8_t buffer[2] = { 0 }, error = 0;
	int16_t value = 0;
	uint8_t newConfig = CONFIG_BASE | CONFIG_CONVT_ONE | CONFIG_MODE_DIFF;
	switch (posCh) {
	case 0:
		newConfig |= CONFIG_SELECT_DIFF01;
		break;
	case 1:
		newConfig |= CONFIG_SELECT_DIFF10;
		break;
	case 2:
		newConfig |= CONFIG_SELECT_DIFF23;
		break;
	case 3:
		newConfig |= CONFIG_SELECT_DIFF32;
		break;
	default:
		newConfig |= CONFIG_SELECT_DIFF01;
		break;
	}

	// Write new setup and configuration
	buffer[0] = _setup | SETUP_BIPOLAR;
	buffer[1] = newConfig;
	TWI_START;
	error += Treo_TWI.write(buffer, 2, 1);
	TWI_STOP;

	// Read differential channel
	TWI_START;
	error += Treo_TWI.read(buffer, 2, 1);
	TWI_STOP;
	value = (int16_t) ((uint16_t) buffer[0] << 8) | buffer[1];

	// Restore setup and configuration
	buffer[0] = _config;
	buffer[1] = _setup;
	TWI_START;
	error += Treo_TWI.write(buffer, 2, 1);
	TWI_STOP;

	return (!error ? value : -1);
}

int NightShade_Treo_MAX11612::readDiffCh0Ch1() {
	return NightShade_Treo_MAX11612::readDifferentialChannel(0);
}

int NightShade_Treo_MAX11612::readDiffCh1Ch0() {
	return NightShade_Treo_MAX11612::readDifferentialChannel(1);
}

int NightShade_Treo_MAX11612::readDiffCh2Ch3() {
	return NightShade_Treo_MAX11612::readDifferentialChannel(2);
}

int NightShade_Treo_MAX11612::readDiffCh3Ch2() {
	return NightShade_Treo_MAX11612::readDifferentialChannel(3);
}

int NightShade_Treo_MAX11612::enableExternalReference(int enable) {
	if (enable) {
		_setup &= ~SETUP_INTREF_ON;
		_setup &= ~SETUP_INT_REF;
		_setup |= SETUP_REF_INPUT;
	} else {
		_setup |= SETUP_INTREF_ON;
		_setup |= SETUP_INT_REF;
		_setup &= ~SETUP_REF_INPUT;
	}
	TWI_START;
	int ret = Treo_TWI.write(&_setup, 1, 1);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MAX11612::enableReferenceOutput(int enable) {
	if (enable) {
		_setup |= SETUP_INTREF_ON;
		_setup |= SETUP_INT_REF;
		_setup |= SETUP_REF_INPUT;
	} else {
		_setup |= SETUP_INTREF_ON;
		_setup |= SETUP_INT_REF;
		_setup &= ~SETUP_REF_INPUT;
	}
	TWI_START;
	int ret = Treo_TWI.write(&_setup, 1, 1);
	TWI_STOP;
	return ret;
}
