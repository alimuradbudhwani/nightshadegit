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
 * MAX11644.h
 *
 *  Created on: Oct 24, 2019
 *      Author: Aaron D. Liebold
 */

#ifndef MAX11644_H_
#define MAX11644_H_

#include <Treo/Drivers/NightShade_Treo_Driver.h>

#define MAX11644_DEFAULT_SLAVEADDR 0x36
#define MAX11644_DEFAULT_CLOCKSPEED 400000

class NightShade_Treo_MAX11644 {
public:
	NightShade_Treo_MAX11644(int port, uint8_t slaveAddress, uint32_t clockSpeed);
	NightShade_Treo_MAX11644(int port);
	int begin();

	int acquireAllChannels();
	uint16_t retrieveChannelData(int channel);
	uint16_t acquireSingleChannel(int channel);
	int16_t acquireDifferentialChannel(int positveChannel);
	int16_t acquireDifferentialChannel();

private:
	int _port;
	uint32_t _clockSpeed;
	uint8_t _slaveAddress;
	uint16_t _adcValues[2];
};


#endif /* MAX11644_H_ */
