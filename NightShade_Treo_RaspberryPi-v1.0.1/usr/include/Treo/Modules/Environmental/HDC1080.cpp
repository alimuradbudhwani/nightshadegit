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
 * HDC1080.cpp
 *
 *  Created on: Oct 14, 2019
 *      Author: Aaron D. Liebold
 */

#include "HDC1080.h"


#define TWI_START	Treo_TWI.begin(_port, _slaveAddress, _clockSpeed)
#define TWI_STOP	Treo_TWI.end();

NightShade_Treo_HDC1080::NightShade_Treo_HDC1080(int port, uint8_t slaveAddress,
		uint32_t clockSpeed) {
	_port = port;
	_slaveAddress = slaveAddress;
	_clockSpeed = clockSpeed;
	_temperature = 0;
	_tempResolution = 0;
	_humidity = 0;
	_humidResolution = 0;

}

NightShade_Treo_HDC1080::NightShade_Treo_HDC1080(int port) {
	_port = port;
	_slaveAddress = HDC1080_DEFAULT_SLAVEADDR;
	_clockSpeed = HDC1080_DEFAULT_CLOCKSPEED;
	_temperature = 0;
	_tempResolution = 0;
	_humidity = 0;
	_humidResolution = 0;
}

int NightShade_Treo_HDC1080::begin() {
	return this->reset();
}

int NightShade_Treo_HDC1080::reset() {
	TWI_START;
	int ret = Treo_TWI.writeWord(REG_CONFIGURATION, 0x8000, 1);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_HDC1080::enableHeater(int enable) {
	TWI_START;
	uint16_t reg = Treo_TWI.readWord(REG_CONFIGURATION, 1);
	reg = enable ? reg | (1 << 13) : reg & ~(1 << 13);
	int ret = Treo_TWI.writeWord(REG_CONFIGURATION, reg, 1);
	TWI_STOP;
	return ret;
}

int NightShade_Treo_HDC1080::setTemperatureResolution(uint8_t setting) {
	TWI_START;
	uint16_t reg = Treo_TWI.readWord(REG_CONFIGURATION, 1);
	reg = setting ? reg | (1 << 10) : reg & ~(1 << 10);
	int ret = Treo_TWI.writeWord(REG_CONFIGURATION, reg, 1);
	TWI_STOP;
	_tempResolution = setting;
	return ret;
}

int NightShade_Treo_HDC1080::setHumidityResolution(uint8_t setting) {
	TWI_START;
	uint16_t reg = Treo_TWI.readWord(REG_CONFIGURATION, 1);
	reg &= ~(3 << 8); // Clear bits
	reg |= ((setting & 0x03) << 8);
	int ret = Treo_TWI.writeWord(REG_CONFIGURATION, reg, 1);
	TWI_STOP;
	_humidResolution = setting;
	return ret;
}

int NightShade_Treo_HDC1080::acquireData(int readTemp, int readHumid) {
	int ret = 0;
	TWI_START;

	// Set Acquisition Mode
	uint16_t reg = Treo_TWI.readWord(REG_CONFIGURATION, 1);
	reg = (readTemp && readHumid) ? reg | (1 << 12) : reg & ~(1 << 12);
	ret = Treo_TWI.writeWord(REG_CONFIGURATION, reg, 1);

	// Start conversion
	if (readTemp) {
		uint8_t buffer[1] = {0x00};
		ret += Treo_TWI.write(buffer, 1, 1);
	} else {
		uint8_t buffer[1] = {0x01};
		ret += Treo_TWI.write(buffer, 1, 1);
	}

	int delay = 1000;

	if (readTemp) {
		if (_tempResolution == 0) delay += 6350;
		else delay += 3650;
	}

	if (readHumid) {
		if (_humidResolution == 0) {
			delay += 6500;
		} else if (_humidResolution == 1) {
			delay += 3850;
		} else {
			delay += 2500;
		}
	}

	Treo_Delayus(delay); // Wait for conversion

//	if (readTemp) _temperature = (uint16_t) Treo_TWI.readWord(REG_TEMPERATURE, 1);
//	if (readHumid) _humidity = (uint16_t) Treo_TWI.readWord(REG_HUMIDITY, 1);

	uint8_t buffer[2];
	if (readTemp && readHumid) {
		ret += Treo_TWI.read(buffer,4,1);
		_temperature = (buffer[0] << 8) | buffer[1];
		_humidity = (buffer[2] << 8) | buffer[3];
	} else if (readTemp) {
		ret += Treo_TWI.read(buffer,2,1);
		_temperature = (buffer[0] << 8) | buffer[1];
	} else if (readHumid) {
		ret += Treo_TWI.read(buffer,2,1);
		_humidity = (buffer[0] << 8) | buffer[1];
	}

	TWI_STOP;
	return ret;
}

uint16_t NightShade_Treo_HDC1080::readTemperatureRaw() {
	return _temperature;
}

float NightShade_Treo_HDC1080::readTemperature() {
	return (float) _temperature * 165.0 / 65536 - 40;
}

uint16_t NightShade_Treo_HDC1080::readHumidityRaw() {
	return _humidity;
}

float NightShade_Treo_HDC1080::readHumidity() {
	return (float) _humidity * 100.0 / 65536;
}

uint64_t NightShade_Treo_HDC1080::readSerialId() {
	uint64_t serial = (uint64_t) Treo_TWI.readWord(REG_SERIAL_ID1, 1) << 24;
	serial |= (uint64_t) Treo_TWI.readWord(REG_SERIAL_ID2, 1) << 8;
	serial |= (uint64_t) Treo_TWI.readWord(REG_SERIAL_ID3) & 0xFF;
	return serial;
}

uint16_t NightShade_Treo_HDC1080::readManufacturerId() {
	return Treo_TWI.readWord(REG_MANUFACTURER_ID, 1);
}

uint16_t NightShade_Treo_HDC1080::readDeviceId() {
	return Treo_TWI.readWord(REG_DEVICE_ID, 1);
}

