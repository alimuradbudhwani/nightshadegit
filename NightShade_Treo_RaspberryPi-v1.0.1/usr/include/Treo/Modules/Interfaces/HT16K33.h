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
 * HT16K33.h
 *
 *  Created on: May 28, 2020
 *      Author: Aaron D. Liebold
 */

#ifndef HT16K33_H_
#define HT16K33_H_

#include <Treo/Drivers/NightShade_Treo_Driver.h>

#define HT16K33_DEFAULT_SLAVEADDR	0x70
#define HT16K33_DEFAULT_CLOCKSPEED	400000u

#define HT16K33_REG_DATA_PTR	0x00
#define HT16K33_REG_SYS_SLP		0x20
#define HT16K33_REG_SYS_WAKE	0x21
#define HT16K33_REG_KEY_PTR		0x40
#define HT16K33_REG_INT_FLG_PTR	0x60
#define HT16K33_REG_DISP_SETUP	0x80
#define HT16K33_REG_INT_SETUP	0xA0
#define HT16K33_REG_DIM_SET		0xE0

// Character definitions
#define CHARACTER_NULL	0x00
#define	CHARACTER_PLUS_DECIMAL	(1 << 7)

class NightShade_Treo_HT16K33 {
public:
	NightShade_Treo_HT16K33(int port,
			uint8_t slaveAddress, uint32_t clockSpeed);
	NightShade_Treo_HT16K33(int port);

	int begin();
	int sleep(int enable);
	int setIntPin(int enableInt, int activeHigh);
	void setDigit(int digit, int8_t value);
	void setDecimal(int digit, int enable);
	void printNumber(double value, int decimalPlaces, int offset);
	void printNumber(double value, int decimalPlaces);
	void printNumber(double value);
	void clearDigits();
	int writeDisplay();
	int setOnBlinking(int screenEnable, int blinkingValue);
	int setBrightness(uint8_t brightness);

private:
	int _port;
	uint8_t _slaveAddress;
	uint32_t _clockSpeed;
	int8_t _display[16];
	const uint8_t _characters[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D,
			0x07, 0x7F, 0x67 };
};

#endif /* HT16K33_H_ */
