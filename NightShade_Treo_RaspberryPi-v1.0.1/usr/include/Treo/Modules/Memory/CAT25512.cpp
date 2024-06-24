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
 *  CAT25512.cpp
 *
 *  Created on: May 29, 2019
 *      Author: Aaron D. Liebold
 */

#include "CAT25512.h"

#define SPI_START	Treo_SPI.begin(this->_spiPort, this->_chipSelectPin, this->_spiBaudrate, CAT25512_SPI_MODE, CAT25512_BITS_PER_WORD, CAT25512_BITORDER_LSBFIRST)
#define SPI_STOP	Treo_SPI.end()

NightShade_Treo_CAT25512::NightShade_Treo_CAT25512(int spiPort,
		int chipSelectPin, uint32_t spiClockSpeed) {
	_spiPort = spiPort;
	_chipSelectPin = chipSelectPin;
	_spiBaudrate = spiClockSpeed;
	_writeAddress = 0;
}

NightShade_Treo_CAT25512::NightShade_Treo_CAT25512(int spiPort,
		int chipSelectPin) {
	_spiPort = 0;
	_chipSelectPin = chipSelectPin;
	_spiBaudrate = CAT25512_DEFAULT_CLOCKSPEED;
	_writeAddress = 0;
}

int NightShade_Treo_CAT25512::begin() {
	// Send Write Enable Command
	NightShade_Treo_CAT25512::attachBuffers(txBuffer, rxBuffer);
	return 0;
}

int NightShade_Treo_CAT25512::enableWrite(int enable) {
	uint8_t buffer[2] = { 0 };
	int ret = 0;
	buffer[0] = enable ? CMD_WREN : CMD_WRDI; // WREN or WRDI Command
	SPI_START;
	ret = Treo_SPI.transfer(buffer, 1);
	SPI_STOP;
	Treo_Delay(5);
	if (((NightShade_Treo_CAT25512::readStatusReg() & 0x02) && enable)
			|| (!(NightShade_Treo_CAT25512::readStatusReg() & 0x02) && !enable)) return 0;
	else return -1;
}

int NightShade_Treo_CAT25512::startMemoryWrite(uint16_t startAddress) {
	_writeAddress = startAddress;
	NightShade_Treo_CAT25512::flushTx();
	return NightShade_Treo_CAT25512::enableWrite(1);
}

void NightShade_Treo_CAT25512::endMemoryWrite() {
	uint8_t dataPacket[NightShade_Treo_CAT25512::txAvailable() + 3] = { 0 };
	dataPacket[0] = CMD_WRITE;
	dataPacket[1] = (uint8_t) (_writeAddress >> 8) & 0xFF;
	dataPacket[2] = (uint8_t) _writeAddress & 0xFF;
	for (unsigned int x = 3; x < sizeof(dataPacket); ++x)
		dataPacket[x] = NightShade_Treo_CAT25512::retrieveTxByte();
	SPI_START;
	Treo_SPI.transfer(dataPacket, sizeof(dataPacket));
	SPI_STOP;
	while (NightShade_Treo_CAT25512::isBusy()) {
	}; // Wait for write to complete.
}

int NightShade_Treo_CAT25512::requestMemoryRead(uint16_t startAddress,
		int numberOfBytes) {
	uint8_t received[numberOfBytes + 3] = { 0 };
	received[0] = CMD_READ;
	received[1] = (uint8_t) (startAddress >> 8) & 0xFF;
	received[2] = (uint8_t) startAddress & 0xFF;

	while (NightShade_Treo_CAT25512::isBusy())
		;
	SPI_START;
	int ret = Treo_SPI.transfer(received, sizeof(received));
	SPI_STOP;

	NightShade_Treo_CAT25512::loadRxBuffer((char*) received, 3, numberOfBytes);
	return ret;
}

int NightShade_Treo_CAT25512::enableHardwareWriteProtection(int enable) {
	uint8_t status = NightShade_Treo_CAT25512::readStatusReg();
	status = enable ? (status | SRBIT_WPEN) : (status & ~SRBIT_WPEN);
	return NightShade_Treo_CAT25512::writeStatusReg(status);
}

int NightShade_Treo_CAT25512::enableIdentificationPage(int enable) {
	uint8_t status = NightShade_Treo_CAT25512::readStatusReg();
	status = enable ? (status | SRBIT_IPL) : (status & ~SRBIT_IPL);
	return NightShade_Treo_CAT25512::writeStatusReg(status);
}

int NightShade_Treo_CAT25512::permanentlyLockIdentificationPage(int enable) {
	uint8_t status = NightShade_Treo_CAT25512::readStatusReg();
	status = enable ? (status | SRBIT_LIP) : (status & ~SRBIT_LIP);
	return NightShade_Treo_CAT25512::writeStatusReg(status);
}

int NightShade_Treo_CAT25512::setProtectionLevel(int protectionLevel) {
	uint8_t protectionBits;
	uint8_t status = NightShade_Treo_CAT25512::readStatusReg();

	switch ((uint8_t) protectionLevel) {
	case 0:
		protectionBits = SRBIT_PROT0;
		break;
	case 1:
		protectionBits = SRBIT_PROT1;
		break;
	case 2:
		protectionBits = SRBIT_PROT2;
		break;
	case 3:
		protectionBits = SRBIT_PROT3;
		break;
	default:
		protectionBits = SRBIT_PROT0;
		break;
	}

	status = status & ~((uint8_t) SRBIT_PROT3); // Clear current protection setting
	status = ((uint8_t) status | protectionBits); // Set new protection level
	return NightShade_Treo_CAT25512::writeStatusReg(status);
}

int NightShade_Treo_CAT25512::writeStatusReg(uint8_t regValue) {
	uint8_t buffer[2] = { 0 };
	buffer[0] = CMD_WRSR;
	buffer[1] = regValue;
	while (NightShade_Treo_CAT25512::isBusy())
		;
	NightShade_Treo_CAT25512::enableWrite(1);
	SPI_START;
	int ret = Treo_SPI.transfer(buffer, 2);
	SPI_STOP;
	Treo_Delay(5); // Wait 5 ms to avoid RDSR command during WRSR command
	return ret;
}

int NightShade_Treo_CAT25512::readStatusReg() {
	uint8_t buffer[2] = { 0 };
	buffer[0] = CMD_RDSR;
	SPI_START;
	uint8_t ret = Treo_SPI.transfer(buffer, 2);
	SPI_STOP;
	if (ret >= 0) return buffer[1];
	else return -1;
}

int NightShade_Treo_CAT25512::isBusy() {
	uint8_t status = NightShade_Treo_CAT25512::readStatusReg();
	return (int) status & 0x01;
}

