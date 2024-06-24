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
 * NumberPad.h
 *
 *  Created on: Nov 12, 2019
 *      Author: Aaron D. Liebold
 */

#ifndef NUMBERPAD_H_
#define NUMBERPAD_H_

#include <Treo/Drivers/NightShade_Treo_Driver.h>
#include <Treo/Modules/IOExpanders/TCA9534A.h>

class NightShade_Treo_NumberPad {
public:
	NightShade_Treo_NumberPad(int port, uint8_t slaveAddress,
			uint32_t clockSpeed);
	NightShade_Treo_NumberPad(int port);
	~NightShade_Treo_NumberPad();
	void begin();

	int read();
	int buttonPressed();
	void connectorFlipped(int isFlipped);

	// ASCII values of the keys
	const uint8_t _numbers[4][4] = {
			{	49,	50,	51,	65	},
			{	52,	53,	54,	66	},
			{	55,	56,	57,	67	},
			{	42,	48,	35,	68	}
	};

private:
	NightShade_Treo_TCA9534A *gpio;
	int _port;
	uint8_t _slaveAddress;
	uint32_t _clockSpeed;
	uint8_t _connectorFlipped;
};

#endif /* NUMBERPAD_H_ */
