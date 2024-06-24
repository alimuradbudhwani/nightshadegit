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
 *  CAT25512.h
 *
 *  Created on: May 29, 2019
 *      Author: Aaron D. Liebold
 */

#ifndef NIGHTSHADE_TREO_CAT25512_H_
#define NIGHTSHADE_TREO_CAT25512_H_

#include <Treo/Drivers/NightShade_Treo_Driver.h>

// SPI Setup
#define CAT25512_SPI_MODE	3
#define CAT25512_BITS_PER_WORD	8u
#define CAT25512_BITORDER_LSBFIRST	0
#define CAT25512_DEFAULT_CLOCKSPEED 5000000u

// Command Codes
#define CMD_WREN	0x06
#define CMD_WRDI	0x04
#define CMD_RDSR	0x05
#define CMD_WRSR	0x01
#define CMD_READ	0x03
#define CMD_WRITE	0x02

// Status Reg Bits
#define SRBIT_WPEN	(1 << 7)
#define SRBIT_IPL	(1 << 6)
#define SRBIT_LIP	(1 << 4)
#define SRBIT_IPL	(1 << 6)
#define SRBIT_PROT0	(0 << 2)
#define SRBIT_PROT1	(1 << 2)
#define SRBIT_PROT2	(2 << 2)
#define SRBIT_PROT3	(3 << 2)
#define SRBIT_WEL	(1 << 1)
#define SRBIT_NRDY	(1 << 0)

class NightShade_Treo_CAT25512 : public fifoBuffer {
public:
	NightShade_Treo_CAT25512(int spiPort, int chipSelectPin, uint32_t spiClockSpeed);
	NightShade_Treo_CAT25512(int spiPort, int chipSelectPin);
	int begin();
	int startMemoryWrite(uint16_t startAddress);
	void endMemoryWrite();
	int requestMemoryRead(uint16_t startAddress, int numberOfBytes);
	int enableHardwareWriteProtection(int enable);
	int enableIdentificationPage(int enable);
	int permanentlyLockIdentificationPage(int enable);
	int setProtectionLevel(int protectionLevel);
	int writeStatusReg(uint8_t regValue);
	int readStatusReg();
private:
	int enableWrite(int enable);
	int isBusy();
	int _spiPort;
	int _chipSelectPin;
	uint32_t _spiBaudrate;
	int _writeAddress;
	char txBuffer[128];
	char rxBuffer[128];
};

#endif /* NIGHTSHADE_TREO_CAT25512_H_ */
