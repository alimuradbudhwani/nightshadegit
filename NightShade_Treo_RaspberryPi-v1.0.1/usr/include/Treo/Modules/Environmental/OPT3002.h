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
 * OPT3002.h
 *
 *  Created on: Sep 12, 2019
 *      Author: Aaron D. Liebold
 */

#include <Treo/Drivers/NightShade_Treo_Driver.h>

#ifndef OPT3002_H_
#define OPT3002_H_

#define OPT3002_DEFAULT_SLAVEADDR 0x45
#define OPT3002_DEFAULT_CLOCKSPEED 400000u

// Register Addresses
#define REG_RESULT		(0x00)
#define	REG_CONFIG		(0x01)
#define	REG_LOWLIMIT	(0x02)
#define	REG_HIGHLIMIT	(0x03)
#define REG_MFG_ID		(0x7E)

// Config Bits
#define CONFIG_FSRANGE_FIELD	(15 << 12)
#define CONFIG_AUTORANGE		(12 << 12)
#define CONFIG_CONV_TIME_LONG	(1 << 11)
#define CONFIG_MODE_SHUTDOWN	(0 << 9)
#define CONFIG_MODE_SINGLESHOT	(1 << 9)
#define CONFIG_MODE_CONTINUOUS	(3 << 9)
#define	CONFIG_OVF_FLAG			(1 << 8)
#define CONFIG_CONV_RDY			(1 << 7)
#define CONFIG_HIGHFIELD_FLAG	(1 << 6)
#define CONFIG_LOWFIELD_FLAG	(1 << 5)
#define CONFIG_LATCH_FIELD		(1 << 4)
#define CONFIG_POLARITY_FIELD	(1 << 3)
#define CONFIG_MASKEXPNT_FIELD	(1 << 2)
#define CONFIG_FAULTCNT_FIELD	(1 << 0)




class NightShade_Treo_OPT3002 {
public:
	// Constuctors & begin
	NightShade_Treo_OPT3002(int port, int slaveAddress, int clockSpeed);
	NightShade_Treo_OPT3002(int port);
	int begin();

	// Register functions
	int 		writeConfigReg(uint16_t regSetting);
	uint16_t 	readConfigReg();
	float 		readLightLevel();
	uint16_t	readLightLevelRaw();
	int 		setHighLimit(uint16_t setting);
	int 		setLowLimit(uint16_t setting);
	uint16_t 	readMfgId();

	// Settings
	int		setFullScaleRange(int setting);
	int		enableLongSampleTime(int enable);
	int		setPowerMode(int setting);
	uint8_t readOverflowFlag();
	uint8_t readHighValueFlag();
	uint8_t readLowValueFlag();
	int 	setIntLatch(int enableLatching);
	int		setIntPolarity(int activeHigh);
	int		enableExponentMask(int enableMask);
	uint8_t readFaultCount();

private:
	int _port;
	int _slaveAddress;
	uint32_t _clockSpeed;
};



#endif /* OPT3002_H_ */
