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
 * OPT3002.cpp
 *
 *  Created on: Sep 12, 2019
 *      Author: Aaron D. Liebold
 */

#include "OPT3002.h"




// TWI Parameters
#define TWI_CLOCK_SPEED 400000u
#define TWI_START Treo_TWI.begin(this->_port, this->_slaveAddress, TWI_CLOCK_SPEED)
#define TWI_STOP Treo_TWI.end()

NightShade_Treo_OPT3002::NightShade_Treo_OPT3002(int port, int slaveAddress,
		int i2cClockSpeed) {
	_port = port;
	_slaveAddress = slaveAddress;
	_clockSpeed = TWI_CLOCK_SPEED;
}

NightShade_Treo_OPT3002::NightShade_Treo_OPT3002(int port) {
	_port = port;
	_slaveAddress = OPT3002_DEFAULT_SLAVEADDR;
	_clockSpeed = OPT3002_DEFAULT_CLOCKSPEED;
}

int NightShade_Treo_OPT3002::begin() {
	int ret = writeConfigReg(CONFIG_AUTORANGE | CONFIG_MODE_CONTINUOUS); // Continuous auto-ranged conversions, 100ms Samples
	return ret;
}

int NightShade_Treo_OPT3002::writeConfigReg(uint16_t regSetting) {
	uint8_t buffer[3] = { REG_CONFIG, ((uint8_t) (regSetting >> 8) & 0xFF),
			((uint8_t) regSetting & 0xFF) };
	TWI_START;
	int ret = Treo_TWI.write(buffer, 3, 1);
	TWI_STOP;
	return ret;
}

uint16_t NightShade_Treo_OPT3002::readConfigReg() {
	uint8_t buffer[2] = { REG_CONFIG, 0 };
	TWI_START;
	Treo_TWI.write(buffer, 1, 0);
	Treo_TWI.read(buffer, 2, 1);
	TWI_STOP;
	return (uint16_t) (buffer[0] << 8) | buffer[1];
}

float NightShade_Treo_OPT3002::readLightLevel() {
	uint8_t buffer[2] = { REG_RESULT, 0 };
	TWI_START;
	Treo_TWI.write(buffer, 1, 1);
	Treo_TWI.read(buffer, 2, 1);
	TWI_STOP;
	double result = (uint32_t) (buffer[0] & 0xF) << 8;
	result += buffer[1];
	result = result * (1 << (buffer[0] >> 4)) * 1.2;
	return (float) result;
}

uint16_t NightShade_Treo_OPT3002::readLightLevelRaw() {
	uint8_t buffer[2] = { REG_RESULT, 0 };
	TWI_START;
	Treo_TWI.write(buffer, 1, 1);
	Treo_TWI.read(buffer, 2, 1);
	TWI_STOP;
	return (uint16_t) (buffer[0] << 8) | buffer[1];
}

int NightShade_Treo_OPT3002::setHighLimit(uint16_t setting) {
	uint8_t buffer[3] = { REG_HIGHLIMIT, ((uint8_t) (setting >> 8) & 0xFF),
			((uint8_t) setting & 0xFF) };
	TWI_START;
	int ret = Treo_TWI.write(buffer, 3, 1);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_OPT3002::setLowLimit(uint16_t setting) {
	uint8_t buffer[3] = { REG_LOWLIMIT, ((uint8_t) (setting >> 8) & 0xFF),
			(setting & 0xFF) };
	TWI_START;
	int ret = Treo_TWI.write(buffer, 3, 1);
	TWI_STOP;
	return ret;
}

uint16_t NightShade_Treo_OPT3002::readMfgId() {
	uint8_t buffer[2] = { REG_MFG_ID, 0 };
	TWI_START;
	Treo_TWI.write(buffer, 1, 1);
	Treo_TWI.read(buffer, 2, 1);
	TWI_STOP;
	return (uint16_t) (buffer[0] << 8) | buffer[1];
}

// Settings
int NightShade_Treo_OPT3002::setFullScaleRange(int setting) {
	uint16_t currentSetting = readConfigReg();
	return writeConfigReg(
			(currentSetting & ~CONFIG_FSRANGE_FIELD) | ((setting & 0x0C) << 12));
}

int NightShade_Treo_OPT3002::enableLongSampleTime(int enable) {
	uint16_t regSetting = readConfigReg();
	regSetting =
			enable ?
					regSetting | CONFIG_CONV_TIME_LONG :
					regSetting & ~CONFIG_CONV_TIME_LONG;
	return writeConfigReg(regSetting);
}

int NightShade_Treo_OPT3002::setPowerMode(int setting) {
	uint16_t regSetting = readConfigReg();
	regSetting &= ~CONFIG_MODE_CONTINUOUS; // Clear Mode
	regSetting |= ((setting & 0x03) << 9); // Set mode
	return writeConfigReg(regSetting);
}

uint8_t NightShade_Treo_OPT3002::readOverflowFlag() {
	if (readConfigReg() & CONFIG_OVF_FLAG) return 1;
	else return 0;
}

uint8_t NightShade_Treo_OPT3002::readHighValueFlag() {
	if (readConfigReg() & CONFIG_HIGHFIELD_FLAG) return 1;
	else return 0;
}

uint8_t NightShade_Treo_OPT3002::readLowValueFlag() {
	if (readConfigReg() & CONFIG_LOWFIELD_FLAG) return 1;
	else return 0;
}

int NightShade_Treo_OPT3002::setIntLatch(int enableLatching) {
	uint16_t regSetting = readConfigReg();
	regSetting =
			enableLatching ?
					(regSetting | CONFIG_LATCH_FIELD) :
					(regSetting & ~CONFIG_LATCH_FIELD);
	return writeConfigReg(regSetting);
}

int NightShade_Treo_OPT3002::setIntPolarity(int activeHigh) {
	uint16_t regSetting = readConfigReg();
	regSetting =
			activeHigh ?
					(regSetting | CONFIG_POLARITY_FIELD) :
					(regSetting & ~CONFIG_POLARITY_FIELD);
	return writeConfigReg(regSetting);
}

int NightShade_Treo_OPT3002::enableExponentMask(int enableMask) {
	uint16_t regSetting = readConfigReg();
	regSetting =
			enableMask ?
					(regSetting | CONFIG_MASKEXPNT_FIELD) :
					(regSetting & ~CONFIG_MASKEXPNT_FIELD);
	return writeConfigReg(regSetting);
}

uint8_t NightShade_Treo_OPT3002::readFaultCount() {
	return (readConfigReg() & 0x03);
}

