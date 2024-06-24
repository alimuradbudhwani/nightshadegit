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
 * ICM20948.cpp
 *
 *  Created on: Sep 17, 2019
 *      Author: Aaron D. Liebold
 */

#include "ICM20948.h"

#define TWI_START Treo_TWI.begin(_port, _slaveAddress, _clockSpeed)
#define TWI_STOP Treo_TWI.end()

NightShade_Treo_ICM20948::NightShade_Treo_ICM20948(int port,
		uint8_t slaveAddress, uint32_t clockSpeed) {
	_port = port;
	_slaveAddress = slaveAddress;
	_clockSpeed = clockSpeed;
	_currentRegBank = REG_BANK_SEL_BANK0;

	_accelDataX = 0;
	_accelDataY = 0;
	_accelDataZ = 0;
	_gyroDataX = 0;
	_gyroDataY = 0;
	_gyroDataZ = 0;
	_tempData = 0;
	_compassDataX = 0;
	_compassDataY = 0;
	_compassDataZ = 0;
	_gyroSensitivity = GYRO_SENSITIVITY_DPS_0;
	_accelSensitivity = ACCEL_SENSITIVITY_G_0;
}

NightShade_Treo_ICM20948::NightShade_Treo_ICM20948(int port) {
	_port = port;
	_slaveAddress = ICM20948_DEFAULT_SLAVEADDRESS;
	_clockSpeed = ICM20948_DEFAULT_CLOCKSPEED;
	_currentRegBank = REG_BANK_SEL_BANK0;

	_accelDataX = 0;
	_accelDataY = 0;
	_accelDataZ = 0;
	_gyroDataX = 0;
	_gyroDataY = 0;
	_gyroDataZ = 0;
	_tempData = 0;
	_compassDataX = 0;
	_compassDataY = 0;
	_compassDataZ = 0;
	_gyroSensitivity = GYRO_SENSITIVITY_DPS_0;
	_accelSensitivity = ACCEL_SENSITIVITY_G_0;
}

int NightShade_Treo_ICM20948::begin() {
	int ret = NightShade_Treo_ICM20948::enableSleep(0); // Wake ICM20948
	ret += NightShade_Treo_ICM20948::enableI2cPassthrough(1);
	ret += magBegin();
	return ret;
}

uint8_t NightShade_Treo_ICM20948::readDeviceId() {
	TWI_START;
	setRegisterBank(0);
	uint8_t id = Treo_TWI.readByte(WHO_AM_I);
	TWI_STOP;
	return id;
}

int NightShade_Treo_ICM20948::enableSleep(int enable) {
	TWI_START;
	setRegisterBank(0);
	uint8_t setting = Treo_TWI.readByte(PWR_MGMT_1);
	setting &= ~(1 << 6); // Clear sleep bit
	if (enable) setting |= (1 << 6);// Set sleep bit
	int ret = Treo_TWI.writeByte(PWR_MGMT_1, setting);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ICM20948::clockSelect(uint8_t clkSource) {
	TWI_START;
	setRegisterBank(0);
	uint8_t setting = Treo_TWI.readByte(PWR_MGMT_1);
	setting &= ~(0x07); // Clear CLKSEL bits
	setting |= (clkSource & 0x07);
	int ret = Treo_TWI.writeByte(PWR_MGMT_1, setting);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ICM20948::enableSensors(int enableAccel, int enableGyro,
		int enableTemp) {
	uint8_t buffer[3] = { PWR_MGMT_1, 0, 0 };
	TWI_START;
	setRegisterBank(0);
	Treo_TWI.write(buffer, 1, 1);
	Treo_TWI.read(buffer, 2, 1);

	buffer[0] &= ~(1 << 3); // Clear Temp Enable
	if (enableTemp) buffer[0] |= (1 << 3); // Set Temp Enable bit
	buffer[1] = enableAccel ? 0x00 : 0x38; // Set DISABLE_ ACCEL bits
	buffer[1] = enableGyro ? buffer[1] : (buffer[1] | 0x07); // Set DISABLE_GYRO bits

	buffer[2] = buffer[1];
	buffer[1] = buffer[0];
	buffer[0] = PWR_MGMT_1;
	int ret = Treo_TWI.write(buffer, 3, 1); // Write PWR_MGMT_1 & PWR_MGMT_2
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ICM20948::enableLowPower(int enable) {
	TWI_START;
	setRegisterBank(0);
	uint8_t setting = Treo_TWI.readByte(PWR_MGMT_1);
	setting &= ~(1 << 5); // Clear low power bit
	if (enable) setting | (1 << 5);// Set low power bit
	int ret = Treo_TWI.writeByte(PWR_MGMT_1, setting);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ICM20948::setupSensors(uint8_t accelSensitivity,
		uint8_t accelFilter, uint8_t gyroSensitivity, uint8_t gyroFilter) {
	TWI_START;
	setRegisterBank(2);

	uint8_t setting = Treo_TWI.readByte(GYRO_CONFIG_1);
	setting &= 0xC0; // Clear GYRO_FS_SEL bits
	setting |= ((gyroSensitivity & 0x03) << 1);// Set GYRO_FS_SEL bits
	setting |= (gyroFilter & 0x07) << 3; // LPF Setting
	setting |= gyroFilter & 0x01; // Enable gyro DLPF
	int ret = Treo_TWI.writeByte(GYRO_CONFIG_1, setting);

	switch (gyroSensitivity & 0x03) {
		case 0:
		_gyroSensitivity = GYRO_SENSITIVITY_DPS_0;
		break;
		case 1:
		_gyroSensitivity = GYRO_SENSITIVITY_DPS_1;
		break;
		case 2:
		_gyroSensitivity = GYRO_SENSITIVITY_DPS_2;
		break;
		case 3:
		_gyroSensitivity = GYRO_SENSITIVITY_DPS_3;
		break;
	}

	setting = Treo_TWI.readByte(ACCEL_CONFIG);
	setting &= 0xC0; // Clear ACCEL_FS_SEL bits
	setting |= ((accelSensitivity & 0x03) << 1);// Set ACCEL_FS_SEL bits
	setting |= (accelFilter & 0x07) << 3; // LPF Setting
	setting |= accelFilter & 0x01; // Enable accel LPF
	ret += Treo_TWI.writeByte(ACCEL_CONFIG, setting);

	switch (accelSensitivity & 0x03) {
		case 0:
		_accelSensitivity = ACCEL_SENSITIVITY_G_0;
		break;
		case 1:
		_accelSensitivity = ACCEL_SENSITIVITY_G_1;
		break;
		case 2:
		_accelSensitivity = ACCEL_SENSITIVITY_G_2;
		break;
		_accelSensitivity = ACCEL_SENSITIVITY_G_3;
		break;
	}
	TWI_STOP;
	return ret;
}

int NightShade_Treo_ICM20948::enableI2cPassthrough(int enable) {
	TWI_START;
	setRegisterBank(0);
	uint8_t reg = Treo_TWI.readByte(INT_PIN_CFG);
	reg = enable ? reg | (1 << 1) : reg & ~(1 << 1);
	int ret = Treo_TWI.writeByte(INT_PIN_CFG, reg);
	TWI_STOP;
	return ret;
}

uint8_t NightShade_Treo_ICM20948::readIntPinCfg() {
	TWI_START;
	setRegisterBank(0);
	uint8_t reg = Treo_TWI.readByte(INT_PIN_CFG);
	TWI_STOP;
	return reg;
}

// Read data
int NightShade_Treo_ICM20948::getData(int getAccelGyroTempData,
		int getCompassData) {
	int ret = 0;

	if (getAccelGyroTempData) {
		// Retrieve accel, gyro, and temp data
		uint8_t buffer[14] = { 0xFF };
		buffer[0] = ACCEL_XOUT_H;

		TWI_START;
		setRegisterBank(0);
		ret += Treo_TWI.write(buffer, 1, 1);
		ret += Treo_TWI.read(buffer, 14, 1);
		TWI_STOP;

		_accelDataX = (int16_t) (buffer[0] << 8) + buffer[1];
		_accelDataY = (int16_t) (buffer[2] << 8) + buffer[3];
		_accelDataZ = (int16_t) (buffer[4] << 8) + buffer[5];
		_gyroDataX = (int16_t) (buffer[6] << 8) + buffer[7];
		_gyroDataY = (int16_t) (buffer[8] << 8) + buffer[9];
		_gyroDataZ = (int16_t) (buffer[10] << 8) + buffer[11];
		_tempData = (int16_t) (buffer[12] << 8) + buffer[13];
	}
	if (getCompassData) {
		ret += magReadData();
	}
	return ret;
}

int NightShade_Treo_ICM20948::readAccelX() {
	return _accelDataX;
}

int NightShade_Treo_ICM20948::readAccelY() {
	return _accelDataY;
}

int NightShade_Treo_ICM20948::readAccelZ() {
	return _accelDataZ;
}

float NightShade_Treo_ICM20948::accelSensitivity() {
	return _accelSensitivity;
}

int NightShade_Treo_ICM20948::readGyroX() {
	return _gyroDataX;
}

int NightShade_Treo_ICM20948::readGyroY() {
	return _gyroDataY;
}

int NightShade_Treo_ICM20948::readGyroZ() {
	return _gyroDataZ;
}

float NightShade_Treo_ICM20948::gyroSensitivity() {
	return _gyroSensitivity;
}

int NightShade_Treo_ICM20948::readCompassX() {
	return _compassDataX;
}

int NightShade_Treo_ICM20948::readCompassY() {
	return _compassDataY;
}

int NightShade_Treo_ICM20948::readCompassZ() {
	return _compassDataZ;
}

int NightShade_Treo_ICM20948::readTemperature() {
	return (float) _tempData / TEMP_SENSITIVITY_HUNDRETH_DEGREE + 2100;
}

int NightShade_Treo_ICM20948::setRegisterBank(int bankNumber) {
	uint8_t regValue;
	int ret = 0;
	switch (bankNumber) {
	case 0:
		regValue = REG_BANK_SEL_BANK0;
		break;
	case 1:
		regValue = REG_BANK_SEL_BANK1;
		break;
	case 2:
		regValue = REG_BANK_SEL_BANK2;
		break;
	case 3:
		regValue = REG_BANK_SEL_BANK3;
		break;
	default:
		regValue = REG_BANK_SEL_BANK0;
		break;
	}
	if (_currentRegBank != regValue) {
		TWI_START;
		ret = Treo_TWI.writeByte(REG_BANK_SEL_ADDR, regValue);
		TWI_STOP;
		_currentRegBank = regValue;
	}
	return ret;
}

// ********** Magnetometer ********** //

int NightShade_Treo_ICM20948::magBegin() {
	return magSetMeasurementMode(8);
}

uint8_t NightShade_Treo_ICM20948::magReadDeviceId() {
	Treo_TWI.begin(_port, MAG_SLAVE_ADDR, _clockSpeed);
	uint8_t id = Treo_TWI.readByte(MAG_WIA2);
	Treo_TWI.end();
	return id;
}

int NightShade_Treo_ICM20948::magDataReady() {
	Treo_TWI.begin(_port, MAG_SLAVE_ADDR, _clockSpeed);
	uint8_t reg = Treo_TWI.readByte(MAG_ST1);
	Treo_TWI.end();
	return reg & 0x01;
}

int NightShade_Treo_ICM20948::magReadData() {
	uint8_t buffer[8] = { 0 };
	int ret = 0;
	do {
		buffer[0] = MAG_HXL;
		ret += Treo_TWI.begin(_port, MAG_SLAVE_ADDR, _clockSpeed);
		ret += Treo_TWI.write(buffer, 1, 1);
		ret += Treo_TWI.read(buffer, 8, 1);
		ret += Treo_TWI.end();
	} while (buffer[7] & 0x08); // Check for Mag Sensor Overflow
	NightShade_Treo_ICM20948::_compassDataX = (int16_t) (buffer[1] << 8)
			| buffer[0];
	NightShade_Treo_ICM20948::_compassDataY = (int16_t) (buffer[3] << 8)
			| buffer[2];
	NightShade_Treo_ICM20948::_compassDataZ = (int16_t) (buffer[5] << 8)
			| buffer[4];
	return ret;
}

// 0=Off, 1=Single, 2=10 Hz, 4=20 Hz, 6=50Hz, 8=100Hz, 16=Self Test Mode
int NightShade_Treo_ICM20948::magSetMeasurementMode(uint8_t mode) {
	int ret = Treo_TWI.begin(_port, MAG_SLAVE_ADDR, _clockSpeed);
	ret += Treo_TWI.writeByte(MAG_CNTL2, mode & 0x1F);
	ret += Treo_TWI.end();
	return ret;
}

int NightShade_Treo_ICM20948::magReset(uint8_t enable) {
	int ret = 0;
	if (enable) {
		ret += Treo_TWI.begin(_port, MAG_SLAVE_ADDR, _clockSpeed);
		ret += Treo_TWI.writeByte(MAG_CNTL3, 1);
		ret += Treo_TWI.end();
	}
	return ret;
}
