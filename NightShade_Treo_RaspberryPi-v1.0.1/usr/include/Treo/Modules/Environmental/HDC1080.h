/*******************************************************************************
 * Copyright (c) 2020, NightShade Electronics - All rights reserved.
 * 
 * BSD 3-Clause License
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/*
 * HDC1080.h
 *
 *  Created on: Oct 14, 2019
 *      Author: Aaron D. Liebold
 */

#ifndef HDC1080_H_
#define HDC1080_H_

#include <Treo/Drivers/NightShade_Treo_Driver.h>

#define HDC1080_DEFAULT_SLAVEADDR 0x40
#define HDC1080_DEFAULT_CLOCKSPEED 400000u

#define REG_TEMPERATURE		0x00
#define REG_HUMIDITY		0x01
#define REG_CONFIGURATION	0x02
#define REG_SERIAL_ID1		0xFB
#define REG_SERIAL_ID2		0xFC
#define REG_SERIAL_ID3		0xFD
#define REG_MANUFACTURER_ID	0xFE
#define REG_DEVICE_ID		0xFF

class NightShade_Treo_HDC1080 {
public:
	NightShade_Treo_HDC1080(int port, uint8_t slaveAddress, uint32_t clockSpeed);
	NightShade_Treo_HDC1080(int port);
	int begin();

	int reset();
	int enableHeater(int enable);
	int setTemperatureResolution(uint8_t setting);
	int setHumidityResolution(uint8_t setting);

	int acquireData(int readTemp, int readHumid);
	uint16_t readTemperatureRaw();
	float readTemperature();
	uint16_t readHumidityRaw();
	float readHumidity();

	uint64_t readSerialId();
	uint16_t readManufacturerId();
	uint16_t readDeviceId();

private:
	int _port;
	uint8_t _slaveAddress;
	uint32_t _clockSpeed;
	uint16_t _temperature;
	uint16_t _humidity;
	uint8_t _tempResolution;
	uint8_t _humidResolution;
};


#endif /* HDC1080_H_ */
