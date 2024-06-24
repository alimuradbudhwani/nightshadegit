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
 * MCP9600.cpp
 *
 *  Created on: Mar 27, 2020
 *      Author: Aaron D. Liebold
 */

#include "MCP9600.h"


#define TWI_START Treo_TWI.begin(this->_port, this->_slaveAddress, this->_clockSpeed)
#define	TWI_STOP Treo_TWI.end()

NightShade_Treo_MCP9600::NightShade_Treo_MCP9600(int port, uint8_t slaveAddress,
		uint32_t clockSpeed) {
	this->_port = port;
	this->_slaveAddress = slaveAddress;
	this->_clockSpeed = clockSpeed;
}

NightShade_Treo_MCP9600::NightShade_Treo_MCP9600(int port) {
	this->_port = port;
	this->_slaveAddress = MCP9600_SLAVEADDR;
	this->_clockSpeed = MCP9600_DEFAULT_CLOCKSPEED;
}

// Setup Functions
int NightShade_Treo_MCP9600::begin() {
	int error = this->setThermocoupleType(0x00); // K-Type
	error += this->setFilterCoefficient(0x01); // Low Filtering
	error += this->setColdJunctionRes(0); // 0.0625 deg res
	error += this->setMeasurementRes(0x01); // 16-bit Resolution
	error += this->setPowerMode(0x00); // Normal Operation
	error += this->setupAlert(1, 1, 100, 0); // Turn off alert 1 interrupt
	error += this->setupAlert(2, 1, 150, 0); // Turn off alert 2 interrupt
	error += this->setupAlert(3, 1, 200, 0); // Turn off alert 3 interrupt
	error += this->setupAlert(4, 1, 300, 0); // Turn off alert 4 interrupt
	return error;
}

int NightShade_Treo_MCP9600::setThermocoupleType(int setting) {
	TWI_START;
	uint8_t reg = this->readByteWithStop(MCP9600_REG_TC_SENSOR_CONF);
	reg &= 0x07;
	reg |= (setting & 0x07) << 4;
	int ret = Treo_TWI.writeByte(MCP9600_REG_TC_SENSOR_CONF, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MCP9600::setFilterCoefficient(int setting) {
	TWI_START;
	uint8_t reg = this->readByteWithStop(MCP9600_REG_TC_SENSOR_CONF);
	reg &= 0xF8;
	reg |= (setting & 0x07);
	int ret = Treo_TWI.writeByte(MCP9600_REG_TC_SENSOR_CONF, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MCP9600::setColdJunctionRes(int setting) {
	TWI_START;
	uint8_t reg = this->readByteWithStop(MCP9600_REG_DEVICE_CONF);
	reg &= 0x7F;
	reg |= (setting ? 0x80 : 0x00);
	int ret = Treo_TWI.writeByte(MCP9600_REG_DEVICE_CONF, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MCP9600::setMeasurementRes(int setting) {
	TWI_START;
	uint8_t reg = this->readByteWithStop(MCP9600_REG_DEVICE_CONF);
	reg &= 0x8F;
	reg |= (setting & 0x07) << 4;
	int ret = Treo_TWI.writeByte(MCP9600_REG_DEVICE_CONF, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MCP9600::setBurstRead(int setting) {
	TWI_START;
	uint8_t reg = this->readByteWithStop(MCP9600_REG_DEVICE_CONF);
	reg &= 0xE0;
	reg |= ((setting & 0x07) << 2) | 0x02; // Set burst samples and burst mode
	int ret = Treo_TWI.writeByte(MCP9600_REG_DEVICE_CONF, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MCP9600::setPowerMode(int setting) {
	TWI_START;
	uint8_t reg = this->readByteWithStop(MCP9600_REG_DEVICE_CONF);
	reg &= 0xFC;
	reg |= (setting & 0x07); // Set shutdown mode
	int ret = Treo_TWI.writeByte(MCP9600_REG_DEVICE_CONF, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MCP9600::setAlertLimit(int alertNumber, int value) {
	uint8_t buffer[3] = { (MCP9600_REG_ALERT1_LIMIT + (alertNumber - 1)),
			((uint8_t) 0xFF & (value >> 6)), ((uint8_t) 0xFF & (value << 2)) };
	TWI_START;
	buffer[0] = (MCP9600_REG_ALERT1_LIMIT + (alertNumber - 1));
	int ret = Treo_TWI.write(buffer, 3, 1);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MCP9600::readAlertLimit(int alertNumber) {
	uint8_t buffer[2];
	buffer[0] = (MCP9600_REG_ALERT1_LIMIT + (alertNumber - 1));
	TWI_START;
	Treo_TWI.write(buffer, 1, 1);
	Treo_TWI.read(buffer, 2, 1);
	TWI_STOP;
	return ((int16_t) (buffer[0] << 8) | buffer[1]) >> 2;
}

int NightShade_Treo_MCP9600::setAlertHyst(int alertNumber, uint8_t value) {
	TWI_START;
	int ret = Treo_TWI.writeByte((MCP9600_REG_ALERT1_HYST + (alertNumber - 1)), value);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MCP9600::setupAlert(int alertNumber, int selectJunction,
		int triggerDirection, int enableAlertOutput) {
	uint8_t reg = 0x82; // Clear int, active low, int mode
	reg |= (selectJunction ? 1 : 0) << 4;
	reg |= (triggerDirection ? 1 : 0) << 3;
	reg |= (enableAlertOutput ? 1 : 0);
	TWI_START;
	int error = Treo_TWI.writeByte(
			(MCP9600_REG_ALERT1_CONF + (alertNumber - 1)), reg);
	TWI_STOP;
	error += this->clearAlertInterrupt(alertNumber);
	return error;
}

int NightShade_Treo_MCP9600::clearAlertInterrupt(int alertNumber) {
	TWI_START;
	uint8_t reg = this->readByteWithStop(MCP9600_REG_ALERT1_CONF + (alertNumber - 1));
	reg |= 0x80; // Clear interrupt
	int ret = Treo_TWI.writeByte((MCP9600_REG_ALERT1_CONF + (alertNumber - 1)), reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MCP9600::clearThUpdateBit() {
	uint8_t reg = this->readStatusReg();
	reg &= 0xBF;
	TWI_START;
	int ret = Treo_TWI.writeByte(MCP9600_REG_STATUS, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_MCP9600::clearBurstCompleteBit() {
	uint8_t reg = this->readStatusReg();
	reg &= 0x7F;
	this->clearBurstCompleteBit();
	TWI_START;
	int ret = Treo_TWI.writeByte(MCP9600_REG_STATUS, reg);
	TWI_STOP;
	return ret;
}

// Read Functions
uint8_t NightShade_Treo_MCP9600::readStatusReg() {
	uint8_t buffer[1] = { MCP9600_REG_STATUS };
	TWI_START;
	Treo_TWI.write(buffer, 1, 1);
	Treo_TWI.read(buffer, 1, 1);
	TWI_STOP;
	return buffer[0];
}

int NightShade_Treo_MCP9600::readTemperature() {
	uint8_t buffer[2];
	buffer[0] = MCP9600_REG_HOT_JUNC_TEMP;
	TWI_START;
	Treo_TWI.write(buffer, 1, 1);
	Treo_TWI.read(buffer, 2, 1);
	TWI_STOP;
	this->clearThUpdateBit(); // Clear the Th conversion complete bit
	return (int16_t) (buffer[0] << 8) | buffer[1];
}

int NightShade_Treo_MCP9600::readColdJuncTemperature() {
	uint8_t buffer[2];
	buffer[0] = MCP9600_REG_COLD_JUNC_TEMP;
	TWI_START;
	Treo_TWI.write(buffer, 1, 1);
	Treo_TWI.read(buffer, 2, 1);
	TWI_STOP;
	return (int16_t) (buffer[0] << 8) | buffer[1];
}

int NightShade_Treo_MCP9600::readDeltaTemperature() {
	uint8_t buffer[2];
	buffer[0] = MCP9600_REG_DELTA_JUNC_TEMP;
	TWI_START;
	Treo_TWI.write(buffer, 1, 1);
	Treo_TWI.read(buffer, 2, 1);
	TWI_STOP;
	return (int16_t) (buffer[0] << 8) | buffer[1];
}

int NightShade_Treo_MCP9600::dataReady() {
	return (this->readStatusReg() & 0x40 ? 1 : 0);
}

int NightShade_Treo_MCP9600::burstReady() {
	return (this->readStatusReg() & 0x80 ? 1 : 0);
}

uint8_t NightShade_Treo_MCP9600::readByteWithStop(uint8_t regAddress) {
	uint8_t buffer[1] = { regAddress };
	TWI_START;
	Treo_TWI.write(buffer, 1, 1);
	Treo_TWI.read(buffer, 1, 1);
	TWI_STOP;
	return buffer[0];
}
