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
 * ADXL345.cpp
 *
 *  Created on: Jun 21, 2019
 *      Author: Aaron D. Liebold
 */

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "ADXL345.h"
/* ----------------END INCLUDES ------------------------------*/

/***********************************************************************
 * MACROS
 ***********************************************************************/


#define TWI_START Treo_TWI.begin(this->_port, this->_slaveAddress, this->_clockSpeed)
#define TWI_STOP Treo_TWI.end()

#define ADXL345_DEFAULT_CLOCKSPEED 400000u
#define ADXL345_DEFAULT_SLAVEADDRESS 0x1D

// Register Addresses
#define REG_DEVID			0x00
#define REG_THRESH_TAP		0x1D
#define REG_OFSX			0x1E
#define REG_OFSY			0x1F
#define REG_OFSZ			0x20
#define REG_DUR				0x21
#define REG_LATENT			0x22
#define REG_WINDOW			0x23
#define REG_THRESH_ACT		0x24
#define REG_THRES_INACT		0x25
#define REG_TIME_INACT		0x26
#define REG_ACT_INACT_CTL	0x27
#define REG_THRESH_FF		0x28
#define REG_TIME_FF			0x29
#define REG_TAP_AXES		0x2A
#define REG_ACT_TAP_STATUS	0x2B
#define REG_BW_RATE			0x2C
#define REG_POWER_CTL		0x2D
#define REG_INT_ENABLE		0x2E
#define REG_INT_MAP			0x2F
#define REG_INT_SOURCE		0x30
#define REG_DATA_FORMAT		0x31
#define REG_DATAX0			0x32
#define REG_DATAX1			0x33
#define REG_DATAY0			0x34
#define REG_DATAY1			0x35
#define REG_DATAZ0			0x36
#define REG_DATAZ1			0x37
#define REG_FIFO_CTL		0x38
#define REG_FIFO_STATUS		0x39
/* ----------------END MACROS ------------------------------*/

/***********************************************************************
 * SETUP
 ***********************************************************************/
NightShade_Treo_ADXL345::NightShade_Treo_ADXL345(int port, uint8_t slaveAddress, uint32_t clockSpeed) {
	this->_port = port;
	this->_slaveAddress = slaveAddress;
	this->_clockSpeed = clockSpeed;
}

NightShade_Treo_ADXL345::NightShade_Treo_ADXL345(int port) {
	this->_port = port;
	this->_slaveAddress = ADXL345_DEFAULT_SLAVEADDRESS;
	this->_clockSpeed = ADXL345_DEFAULT_CLOCKSPEED;
}

int NightShade_Treo_ADXL345::begin() {
	TWI_START;
	int ret = Treo_TWI.writeByte(REG_DATA_FORMAT, 0x21); // 4g Range, Right Justified, Inverted INT
	ret += Treo_TWI.writeByte(REG_POWER_CTL, 0x08);// Measurement Mode Enabled
	TWI_STOP;
	ret += this->writeInterruptMap(0x00);// Map all interrupts to INT1 pin
	ret += this->enableFullResolution(0);
	ret += this->setMeasurementRange(1);
	return ret;
}
/* ----------------END SETUP ------------------------------*/

/***********************************************************************
 * READ DATA
 ***********************************************************************/
int NightShade_Treo_ADXL345::retrieveData() {
	uint8_t available = NightShade_Treo_ADXL345::numberFifoEntries();
	uint8_t buffer[6] = { 6 };
	int ret = 0;
	_xDataAvailable = 0;
	_yDataAvailable = 0;
	_zDataAvailable = 0;
	TWI_START;
	for (int x = 0; x <= available; ++x) {
		buffer[0] = REG_DATAX0;
		ret += Treo_TWI.write(buffer, 1, 0);
		ret += Treo_TWI.read(buffer, 6, 1);
		_xData[_xDataAvailable++] = (int16_t) (buffer[1] << 8) | buffer[0];
		_yData[_yDataAvailable++] = (int16_t) (buffer[3] << 8) | buffer[2];
		_zData[_zDataAvailable++] = (int16_t) (buffer[5] << 8) | buffer[4];
		Treo_Delayus(5);
	}
	TWI_STOP;
	return ret;
}
int NightShade_Treo_ADXL345::readX() {
	if (_xDataAvailable > 0) {
		int data = _xData[0];
		NightShade_Treo_ADXL345::regShiftDown(_xData, _xDataAvailable--);
		return data;
	} else
		return -1;
}
int NightShade_Treo_ADXL345::readY() {
	if (_yDataAvailable > 0) {
		int data = _yData[0];
		NightShade_Treo_ADXL345::regShiftDown(_yData, _yDataAvailable--);
		return data;
	} else
		return -1;
}
int NightShade_Treo_ADXL345::readZ() {
	if (_zDataAvailable > 0) {
		int data = _zData[0];
		NightShade_Treo_ADXL345::regShiftDown(_zData, _zDataAvailable--);
		return data;
	} else
		return -1;
}
/* ----------------END READ DATA ------------------------------*/

/***********************************************************************
 * POWER REG SETTINGS
 ***********************************************************************/
int NightShade_Treo_ADXL345::enableLink(uint8_t enabled) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_POWER_CTL);
	reg = ((reg & 0xDF) | (enabled << 5));
	int ret = Treo_TWI.writeByte(REG_POWER_CTL, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::enableAutoSleep(uint8_t enabled) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_POWER_CTL);
	reg = ((reg & 0xEF) | (enabled << 4));
	int ret = Treo_TWI.writeByte(REG_POWER_CTL, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::enableMeasureMode(uint8_t enabled) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_POWER_CTL);
	reg = ((reg & 0xF7) | (enabled << 3));
	int ret = Treo_TWI.writeByte(REG_POWER_CTL, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::enableSleep(uint8_t enabled) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_POWER_CTL);
	reg = ((reg & 0xFB) | (enabled << 2));
	int ret = Treo_TWI.writeByte(REG_POWER_CTL, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::setWakeUp(uint8_t mode) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_POWER_CTL);
	reg = ((reg & 0xFC) | mode);
	int ret = Treo_TWI.writeByte(REG_POWER_CTL, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::enableLowPower(uint8_t enable) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_BW_RATE);
	reg = ((reg & 0xEF) | enable);
	int ret = Treo_TWI.writeByte(REG_BW_RATE, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::setBandwidth(uint8_t setting) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_BW_RATE);
	reg = ((reg & 0xF0) | setting);
	int ret = Treo_TWI.writeByte(REG_BW_RATE, reg);
	TWI_STOP;
	return ret;
}
/* ----------------END POWER REG SETTINGS ------------------------------*/

/***********************************************************************
 * DATA FORMAT SETTINGS
 ***********************************************************************/
int NightShade_Treo_ADXL345::enableSelfTest(uint8_t enabled) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_DATA_FORMAT);
	reg = ((reg & 0x7F) | (enabled << 7));
	int ret = Treo_TWI.writeByte(REG_DATA_FORMAT, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::setOffsetValues(uint8_t xOffest, uint8_t yOffest, uint8_t zOffest) {
	int ret = Treo_TWI.writeByte(REG_OFSX, xOffest);
	ret += Treo_TWI.writeByte(REG_OFSY, yOffest);
	ret += Treo_TWI.writeByte(REG_OFSZ, zOffest);
	return ret;
}

int NightShade_Treo_ADXL345::invertInterruptOutput(uint8_t enabled) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_DATA_FORMAT);
	reg = ((reg & 0xDF) | (enabled << 5));
	int ret = Treo_TWI.writeByte(REG_DATA_FORMAT, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::enableFullResolution(uint8_t enabled) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_DATA_FORMAT);
	reg = ((reg & 0xF7) | (enabled << 3));
	int ret = Treo_TWI.writeByte(REG_DATA_FORMAT, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::leftJustifyData(uint8_t enabled) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_DATA_FORMAT);
	reg = ((reg & 0xFB) | (enabled << 2));
	int ret = Treo_TWI.writeByte(REG_DATA_FORMAT, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::setMeasurementRange(uint8_t mode) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_DATA_FORMAT);
	reg = ((reg & 0xFC) | mode);
	int ret = Treo_TWI.writeByte(REG_DATA_FORMAT, reg);
	TWI_STOP;
	return ret;
}
/* ----------------END DATA FORMAT SETTINGS ------------------------------*/

/***********************************************************************
 * FIFO SETTINGS
 ***********************************************************************/
int NightShade_Treo_ADXL345::setFifoMode(uint8_t mode) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_FIFO_CTL);
	reg = ((reg & 0x3F) | (mode << 6));
	int ret = Treo_TWI.writeByte(REG_FIFO_CTL, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::enableFifoTrigger(uint8_t enabled) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_FIFO_CTL);
	reg = ((reg & 0xDF) | (enabled << 5));
	int ret = Treo_TWI.writeByte(REG_FIFO_CTL, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::setFifoSamples(uint8_t numberOfSamples) {
	TWI_START;
	uint8_t reg = Treo_TWI.readByte(REG_FIFO_CTL);
	reg = ((reg & 0xE0) | numberOfSamples);
	int ret = Treo_TWI.writeByte(REG_FIFO_CTL, reg);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ADXL345::numberFifoEntries() {
	TWI_START;
	uint8_t entries = (uint8_t) Treo_TWI.readByte(REG_FIFO_STATUS) & 0x3F;
	TWI_STOP;
	return (entries ? 1 : 0);
}

int NightShade_Treo_ADXL345::checkFifoTrigger() {
	TWI_START;
	uint8_t fifoTrigBit = ((uint8_t) Treo_TWI.readByte(REG_FIFO_STATUS) & 0x80) >> 7;
	TWI_STOP;
	return fifoTrigBit;

}
/* ----------------END FIFO SETTINGS ------------------------------*/

/***********************************************************************
 *  INTERRUPT FUNCTIONS
 ***********************************************************************/
int NightShade_Treo_ADXL345::enableInterrupts(uint8_t regValue) {
	return Treo_TWI.writeByte(REG_INT_ENABLE, regValue);
}

int NightShade_Treo_ADXL345::writeInterruptMap(uint8_t regValue) {
	return Treo_TWI.writeByte(REG_INT_MAP, regValue);
}

int NightShade_Treo_ADXL345::readInterruptSources() {
	return Treo_TWI.readByte(REG_INT_SOURCE);
}
/* ----------------END INTERRUPT FUNCTIONS ------------------------------*/

/***********************************************************************
 * TAP FUNCTIONS
 ***********************************************************************/
int NightShade_Treo_ADXL345::setTapThreshold(uint8_t threshold) {
	return Treo_TWI.writeByte(REG_THRESH_TAP, threshold);
}

int NightShade_Treo_ADXL345::setTapDuration(uint8_t duration) {
	return Treo_TWI.writeByte(REG_DUR, duration);
}

int NightShade_Treo_ADXL345::setTapLatency(uint8_t latencyTime) {
	return Treo_TWI.writeByte(REG_LATENT, latencyTime);
}

int NightShade_Treo_ADXL345::setDoubleTapWindow(uint8_t windowTime) {
	return Treo_TWI.writeByte(REG_WINDOW, windowTime);
}

int NightShade_Treo_ADXL345::setActInactControl(uint8_t regValue) {
	return Treo_TWI.writeByte(REG_ACT_INACT_CTL, regValue);
}

int NightShade_Treo_ADXL345::setTapAxes(uint8_t regValue) {
	return Treo_TWI.writeByte(REG_TAP_AXES, regValue);
}

int  NightShade_Treo_ADXL345::readActivityTapStatus() {
	return Treo_TWI.readByte(REG_ACT_TAP_STATUS);
}
/* ----------------END TAP FUNCTIONS ------------------------------*/

/***********************************************************************
 * ACTIVITY / INACTIVITY SETTINGS
 ***********************************************************************/
int NightShade_Treo_ADXL345::setActivityThreshold(uint8_t threshold) {
	return Treo_TWI.writeByte(REG_THRESH_ACT, threshold);
}

int NightShade_Treo_ADXL345::setInactivitiyThreshold(uint8_t threshold) {
	return Treo_TWI.writeByte(REG_THRES_INACT, threshold);
}

int NightShade_Treo_ADXL345::setInactivityTime(uint8_t time) {
	return Treo_TWI.writeByte(REG_TIME_INACT, time);
}
/* ----------------END ACTIVITY / INACTIVITY SETTINGS ------------------------------*/

/***********************************************************************
 * FREE-FALL SETTINGS
 ***********************************************************************/
int NightShade_Treo_ADXL345::setFFThreshold(uint8_t threshold) {
	return Treo_TWI.writeByte(REG_THRESH_FF, threshold);
}

int NightShade_Treo_ADXL345::setFFTime(uint8_t time) {
	return Treo_TWI.writeByte(REG_TIME_FF, time);
}
/* ----------------END FREE-FALL SETTINGS ------------------------------*/

/***********************************************************************
 * PRIVATE FUNCTIONS
 ***********************************************************************/

void NightShade_Treo_ADXL345::regShiftDown(int* buffer, int dataLength) {
	for (int x = 0; x < dataLength - 1; ++x)
		buffer[x] = buffer[x + 1];
}
/* ----------------END PRIVATE FUNCTIONS ------------------------------*/
