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
 * MCP9600.h
 *
 *  Created on: Mar 27, 2020
 *      Author: Aaron D. Liebold
 */

#ifndef MCP9600_H_
#define MCP9600_H_

#include <Treo/Drivers/NightShade_Treo_Driver.h>

#define MCP9600_SLAVEADDR	0x67
#define MCP9600_DEFAULT_CLOCKSPEED 20000 // 20kHz - 100KHz

#define MCP9600_REG_HOT_JUNC_TEMP	0x00
#define MCP9600_REG_DELTA_JUNC_TEMP	0x01
#define MCP9600_REG_COLD_JUNC_TEMP	0x02
#define MCP9600_REG_RAW_ADC			0x03
#define MCP9600_REG_STATUS			0x04
#define MCP9600_REG_TC_SENSOR_CONF	0x05
#define MCP9600_REG_DEVICE_CONF		0x06
#define MCP9600_REG_ALERT1_CONF		0x08
#define MCP9600_REG_ALERT2_CONF		0x09
#define MCP9600_REG_ALERT3_CONF		0x0A
#define MCP9600_REG_ALERT4_CONF		0x0B
#define MCP9600_REG_ALERT1_HYST		0x0C
#define MCP9600_REG_ALERT2_HYST		0x0D
#define MCP9600_REG_ALERT3_HYST		0x0E
#define MCP9600_REG_ALERT4_HYST		0x0F
#define MCP9600_REG_ALERT1_LIMIT	0x10
#define MCP9600_REG_ALERT2_LIMIT	0x11
#define MCP9600_REG_ALERT3_LIMIT	0x12
#define MCP9600_REG_ALERT4_LIMIT	0x13
#define MCP9600_REG_DEVICE_ID		0x20

class NightShade_Treo_MCP9600 {
public:
	NightShade_Treo_MCP9600(int port, uint8_t slaveAddress,
			uint32_t clockSpeed);
	NightShade_Treo_MCP9600(int port);

	// Setup Functions
	int begin();
	int setThermocoupleType(int setting);
	int setFilterCoefficient(int setting);
	int setColdJunctionRes(int setting);
	int setMeasurementRes(int setting);
	int setPowerMode(int setting);
	int setAlertLimit(int alertNumber, int value);
	int setAlertHyst(int alertNumber, uint8_t value);
	int setupAlert(int alertNumber, int selectJunction, int triggerDirection,
			int enableAlertOutput);
	int clearAlertInterrupt(int alertNumber);

	// Read Functions
	int	readAlertLimit(int alertNumber);
	uint8_t readStatusReg();
	int readTemperature();
	int readColdJuncTemperature();
	int readDeltaTemperature();
	int setBurstRead(int setting);
	int burstReady();
	int dataReady();

private:
	int _port;
	uint8_t _slaveAddress;
	uint32_t _clockSpeed;

	int clearThUpdateBit();
	int clearBurstCompleteBit();
	uint8_t readByteWithStop(uint8_t regAddress);
};

#endif /* MCP9600_H_ */
