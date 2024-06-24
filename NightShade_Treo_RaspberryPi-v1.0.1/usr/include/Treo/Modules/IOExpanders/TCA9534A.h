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
 * TCA9534A.h
 *
 *  Created on: Nov 8, 2019
 *      Author: Aaron D. Liebold
 */

#ifndef TCA9534A_H_
#define TCA9534A_H_

#include <Treo/Drivers/NightShade_Treo_Driver.h>

#define TCA9534A_DEFAULT_ADDR 0x3F
#define TCA9534A_DEFAULT_CLOCKSPEED 100000u

#define REGCON_INPUT	0x00
#define REGCON_OUTPUT	0x01
#define REGCON_POLINV	0x02
#define REGCON_CONFIG	0x03

class NightShade_Treo_TCA9534A {
public:
	NightShade_Treo_TCA9534A(int port, uint8_t slaveAddress, uint32_t clockSpeed);
	NightShade_Treo_TCA9534A(int port);
	int begin();

	int setPortMode(uint8_t portMode);
	int setPortOutput(uint8_t portOutput);
	int setPortPolarity(uint8_t portPolarityInversion);

	int setPinMode(int pin, int mode);
	int setPinOutput(int pin, int output);
	int setPinPoarity(int pin, int polarityInverted);

	uint8_t readPortMode();
	uint8_t readPortOutput();
	uint8_t readPortInput();
	uint8_t readPortPolarity();

	int readPinMode(int pin);
	int readPinOutput(int pin);
	int readPinInput(int pin);
	int readPinPolarity(int pin);

private:
	int _port;
	uint8_t _slaveAddress;
	uint32_t _clockSpeed;
};


#endif /* TCA9534A_H_ */
