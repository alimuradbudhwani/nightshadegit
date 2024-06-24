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
 *  MAX11612.h
 *
 *  Created on: July 16, 2019
 *      Author: Aaron D. Liebold
 */

#include <stdint.h>
#include <Treo/Drivers/NightShade_Treo_Driver.h>

#ifndef NIGHTSHADE_TREO_MAX11612_H_
#define NIGHTSHADE_TREO_MAX11612_H_

// TWI Setup
#define MAX11612_SLAVE_ADDR 0x34
#define MAX11612_DEFAULT_CLOCKSPEED 400000u

// Setup Byte Values
#define SETUP_BASE				(1 << 7)
#define SETUP_INT_REF 			(1 << 6)
#define SETUP_REF_INPUT			(1 << 5)
#define SETUP_INTREF_ON			(1 << 4)
#define SETUP_EXT_CLOCK 		(1 << 3)
#define SETUP_BIPOLAR			(1 << 2)
#define SETUP_NO_RESET			(1 << 1)

// Configure Byte Values
#define CONFIG_BASE				(0 << 7)
#define CONFIG_CONVT_ALL		(0 << 5)
#define CONFIG_CONVT_ONE 		(1 << 5)
#define CONFIG_CONVT_UPPER		(2 << 5)
#define CONFIG_CONVT_SELECT		(3 << 5)
#define CONFIG_MODE_SNGLEND		(1 << 0)
#define CONFIG_SELECT_CH0		(0 << 1)
#define CONFIG_SELECT_CH1		(1 << 1)
#define CONFIG_SELECT_CH2		(2 << 1)
#define CONFIG_SELECT_CH3		(3 << 1)
#define CONFIG_MODE_DIFF		(0 << 0)
#define CONFIG_SELECT_DIFF01	(0 << 1)
#define CONFIG_SELECT_DIFF10	(1 << 1)
#define CONFIG_SELECT_DIFF23	(2 << 1)
#define CONFIG_SELECT_DIFF32	(3 << 1)

class NightShade_Treo_MAX11612 {
public:
	// Constructors and begin
	NightShade_Treo_MAX11612(int port, uint32_t clockSpeed);
	NightShade_Treo_MAX11612(int port);
	int begin();
	int acquireAllChannels();
	int readChannel(int channel);
	int readDiffCh0Ch1();
	int readDiffCh1Ch0();
	int readDiffCh2Ch3();
	int readDiffCh3Ch2();
	int enableExternalReference(int enable);
	int enableReferenceOutput(int enable);
private:
	int readDifferentialChannel(int posCh);
	uint8_t _port = 0;
	uint32_t _clockSpeed = 0;
	uint8_t _setup = 0;
	uint8_t _config = 0;
	uint16_t _adcValues[4] = { 0 };
};

#endif /* NIGHTSHADE_TREO_MAX11612_H_ */
