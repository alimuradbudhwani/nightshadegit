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
 * LIS3MDL.cpp
 *
 *  Created on: Mar 25, 2020
 *      Author: Aaron D. Liebold
 */

#include "LIS3MDL.h"


#define TWI_START	Treo_TWI.begin(this->_port, this->_slaveAddr, this->_clockSpeed)
#define	TWI_STOP	Treo_TWI.end()

NightShade_Treo_LIS3MDL::NightShade_Treo_LIS3MDL(int port, uint8_t slaveAddress,
		uint32_t clockSpeed) {
	this->_port = port;
	this->_slaveAddr = slaveAddress;
	this->_clockSpeed = clockSpeed;
	this->_operatingMode = 0;
	this->_oneGaussValue = 0;
	this->_measurementMode = 0;
}

NightShade_Treo_LIS3MDL::NightShade_Treo_LIS3MDL(int port) {
	this->_port = port;
	this->_slaveAddr = LIS3MDL_SLAVE_ADDR;
	this->_clockSpeed = LIS3MDL_DEFAULT_CLOCK_SPEED;
	this->_operatingMode = 0;
	this->_oneGaussValue = 0;
	this->_measurementMode = 0;
}

// Setup Functions
int NightShade_Treo_LIS3MDL::begin() {
	int error = this->setOutputDataRate(7);		// 80 Hz
	error += this->setOperatingMode(3, 3);	// UHP Mag & Temp
	error += this->setFullScaleRange(0);		// 4 Gauss Range
	error += this->setMeasurementMode(1); // Single Conversion Mode
	error += this->enableTemperature(1);
	return error;
}

int NightShade_Treo_LIS3MDL::setOutputDataRate(int setting) {
	uint8_t _setting = (uint8_t) setting > 7 ? 15 : setting;
	TWI_START;
	// Write ODR
	uint8_t reg = Treo_TWI.readByte(LIS3MDL_REG_CTRL_REG1);
	reg &= 0xE1; // Clear DO & FAST_ODR bits
	reg |= ((_setting & 0x07) << 2);
	if (_setting > 7) reg |= 0x02; // Set FAST_ODR
	int ret =Treo_TWI.writeByte(LIS3MDL_REG_CTRL_REG1, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_LIS3MDL::setOperatingMode(int xyMode, int zMode) {
	TWI_START;

	// Write XY OM
	uint8_t reg = Treo_TWI.readByte(LIS3MDL_REG_CTRL_REG1);
	reg &= 0x9F;// Clear OMXY bits
	reg |= ((xyMode & 0x03) << 5);
	int ret = Treo_TWI.writeByte(LIS3MDL_REG_CTRL_REG1, reg);

	// Write Z OM
	reg = Treo_TWI.readByte(LIS3MDL_REG_CTRL_REG4);
	reg &= 0x00;// Clear OMZ bits
	reg |= ((zMode & 0x03) << 2);
	ret += Treo_TWI.writeByte(LIS3MDL_REG_CTRL_REG4, reg);

	TWI_STOP;

	return ret;
}

int NightShade_Treo_LIS3MDL::setMeasurementMode(int setting) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(LIS3MDL_REG_CTRL_REG3);
	reg &= 0x24;
	reg |= (setting & 0x03);
	int ret = Treo_TWI.writeByte(LIS3MDL_REG_CTRL_REG3, reg);
	TWI_STOP;
	ret += this->_measurementMode = (setting & 0x03);
	return ret;
}

int NightShade_Treo_LIS3MDL::enableTemperature(int enable) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(LIS3MDL_REG_CTRL_REG1);
	reg &= 0x7F;
	if (enable) reg |= 0x80;
	int ret = Treo_TWI.writeByte(LIS3MDL_REG_CTRL_REG1, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_LIS3MDL::setFullScaleRange(int setting) {
	TWI_START;
	// Write ODR
	uint8_t reg = Treo_TWI.readByte(LIS3MDL_REG_CTRL_REG2);
	reg &= 0x9F;// Clear FS bits
	reg |= ((setting & 0x03) << 5);
	int ret = Treo_TWI.writeByte(LIS3MDL_REG_CTRL_REG2, reg);
	TWI_STOP;

	switch (setting) {
		case 3:
		_oneGaussValue = 1711;
		break;
		case 2:
		_oneGaussValue = 2281;
		break;
		case 1:
		_oneGaussValue = 3421;
		break;
		case 0:
		default:
		_oneGaussValue = 6842;
		break;
	}
	return ret;
}

int NightShade_Treo_LIS3MDL::enableInterrupt(int enableIntX, int enableIntY,
		int enableIntZ) {
	TWI_START;
	uint8_t reg = (enableIntX || enableIntY || enableIntZ) ? 0x0B : 0x00;
	reg |= (enableIntX ? 0x80 : 0x00);
	reg |= (enableIntY ? 0x40 : 0x00);
	reg |= (enableIntZ ? 0x20 : 0x00);
	int ret = Treo_TWI.writeByte(LIS3MDL_REG_CTRL_REG1, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_LIS3MDL::setInterruptThreshold(int threshold) {
	uint8_t buffer[0];
	buffer[0] = LIS3MDL_REG_INT_THS_L;
	buffer[2] = (uint8_t) (threshold >> 8) & 0xFF;
	buffer[3] = (uint8_t) threshold & 0xFF;
	TWI_START;
	int ret = Treo_TWI.write(buffer, 3, 1);
	TWI_STOP;
	return ret;
}

uint8_t NightShade_Treo_LIS3MDL::readInterruptFlags() {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(LIS3MDL_REG_INT_SRC);
	TWI_STOP;
	return reg;
}

// Data Functions
int NightShade_Treo_LIS3MDL::acquireMagData() {
	uint8_t buffer[6];
	if (this->_measurementMode > 0) this->setMeasurementMode(0x01); // Run single conversion
	while (!this->dataReady())
		;
	TWI_START;
		buffer[0] = 0x29;
		int ret = Treo_TWI.write(buffer, 1, 0);
		ret += Treo_TWI.read(buffer, 6, 1);
		_data[0] = (int16_t) (buffer[0] << 8) | buffer[1];
		_data[1] = (int16_t) (buffer[2] << 8) | buffer[3];
		_data[2] = (int16_t) (buffer[4] << 8) | buffer[5];
	TWI_STOP;
	return ret;
}

int NightShade_Treo_LIS3MDL::readX() {
	return this->_data[0];
}

int NightShade_Treo_LIS3MDL::readY() {
	return this->_data[1];
}

int NightShade_Treo_LIS3MDL::readZ() {
	return this->_data[2];
}

int NightShade_Treo_LIS3MDL::readTemp() {
	TWI_START;
	int temp = Treo_TWI.readWord(LIS3MDL_REG_TEMP_OUT_L);
	TWI_STOP;
	return (temp*10 + 2000) / 8;
}

// Utility Functions
uint8_t NightShade_Treo_LIS3MDL::deviceId() {
	TWI_START;
	uint8_t value = Treo_TWI.readByte(LIS3MDL_REG_WHO_AM_I);
	TWI_STOP;
	return value;
}

int NightShade_Treo_LIS3MDL::enableSelfTest(int enable) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(LIS3MDL_REG_CTRL_REG1);
	reg = enable ? (reg | 0x01) : (reg & 0xFE);
	int ret = Treo_TWI.writeByte(LIS3MDL_REG_CTRL_REG1, reg);
	TWI_STOP;
}

int NightShade_Treo_LIS3MDL::oneGaussValue() {
	return _oneGaussValue;
}

int NightShade_Treo_LIS3MDL::dataReady() {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(LIS3MDL_REG_STATUS_REG);
	TWI_STOP;
	return ((reg & 0x08) ? 1 : 0);
}

int NightShade_Treo_LIS3MDL::rebootMemory() {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(LIS3MDL_REG_CTRL_REG2);
	reg |= 0x08;
	int ret = Treo_TWI.writeByte(LIS3MDL_REG_CTRL_REG2, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_LIS3MDL::restart() {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(LIS3MDL_REG_CTRL_REG2);
	reg |= 0x04;
	int ret = Treo_TWI.writeByte(LIS3MDL_REG_CTRL_REG2, reg);
	TWI_STOP;
	return ret;
}
