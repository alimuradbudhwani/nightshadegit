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
 * NightShade_Treo_Driver.h
 *
 *  Created on: Oct 29, 2018
 *      Author: Aaron D. Liebold
 */

#ifndef NIGHTSHADE_TREO_DRIVER_H_
#define NIGHTSHADE_TREO_DRIVER_H_

#include <stdint.h>

#define TREO_I2C_DEFAULT_CLOCKSPEED	 400000u	// 400kHz
#define TREO_SPI_DEFAULT_CLOCKSPEED	4000000u	//   4MHz
// GPIO Modes
#define GPIO_OUTPUT 1
#define GPIO_INPUT 0
// GPIO States
#define GPIO_HIGH 1
#define GPIO_LOW 0

// General Functions
void Treo_Delay(unsigned long delay_ms);
void Treo_Delayus(unsigned long delay_us);

// TWI/I2C Interface ba
class NightShade_Treo_TWI {
public:
	NightShade_Treo_TWI();

	// Functions return 0 on success
	int begin(int port, uint8_t sevenBitAddress, uint32_t clockSpeed);
	int write(uint8_t *buffer, int numberOfBytes, int sendStop);
	int read(uint8_t *buffer, int numberOfBytes, int sendStop);
	int end();

	int writeByte(uint8_t regAddress, uint8_t byteValue) {
		uint8_t buffer[2] = { regAddress, byteValue };
		return NightShade_Treo_TWI::write(buffer, 2, 1);
	}

	int readByte(uint8_t regAddress) {
		uint8_t buffer[1] = { regAddress };
		NightShade_Treo_TWI::write(buffer, 1, 0);
		NightShade_Treo_TWI::read(buffer, 1, 1);
		return buffer[0];
	}

	int writeWord(uint8_t regAddress, uint16_t wordValue, int isBigEndian) {
		uint8_t buffer[3] = { regAddress, 0, 0 };
		if (isBigEndian) {
			buffer[1] = (wordValue >> 8) & 0xFF;
			buffer[2] = wordValue & 0xFF;
		} else { // Little Endian
			buffer[1] = wordValue & 0xFF;
			buffer[2] = (wordValue >> 8) & 0xFF;
		}
		return NightShade_Treo_TWI::write(buffer, 3, 1);
	}
	int writeWord(uint8_t regAddress, uint16_t wordValue) {
		return writeWord(regAddress, wordValue, 0);
	}

	uint16_t readWord(uint8_t regAddress, int isBigEndian) {
		uint8_t buffer[2] = { regAddress, 0 };
		NightShade_Treo_TWI::write(buffer, 1, 0);
		NightShade_Treo_TWI::read(buffer, 2, 1);
		if (isBigEndian) return (uint16_t) (buffer[0] << 8) | buffer[1]; // Big Endian
		else return (uint16_t) (buffer[1] << 8) | buffer[0]; // Little Endian
	}
	uint16_t readWord(uint8_t regAddress) {
		return readWord(regAddress, 0);
	}

private:
	int _device;
	int _port;
	uint8_t _slaveAddress;
	uint32_t _clockSpeed;
};

// GPIO Interface
class NightShade_Treo_GPIO {
public:
	NightShade_Treo_GPIO();
	void setup(int pin, int mode);
	void write(int pin, int state);
	int read(int pin);

};

// SPI Interface
class NightShade_Treo_SPI {
public:
	NightShade_Treo_SPI();

	// Functions return 0 on success and -1 on failure
	int begin(int port, int chipSelectPin, uint32_t maxBaudrate, int spiMode,
			int bitsPerWord, int lsbFirst);
	int transfer(uint8_t *buffer, int numberOfBytes);
	int end();
private:
	int _device;
	int _port;
	int _csPin;
	int _spiMode;
	int _bitsPerWord;
	uint32_t _clockSpeed;
	int _lsbFirst;
};

// UART Interface
class NightShade_Treo_UART {
public:
	NightShade_Treo_UART();

	void begin(int port, uint32_t baudrate, uint8_t dataBits, uint8_t parity,
			uint8_t endBits);
	int write(uint8_t *buffer, int numberOfBytes);
	int read(uint8_t *buffer, int numberOfBytes);
	int end();
private:
	int _port;
	int _device;
	uint32_t _baudrate;
	int _parity;
	int _dataBits;
	int _endBits;

};

// FIFO Buffer tools
class fifoBuffer {
public:
	void write(int byte) {
		_txBuffer[_txAvailable++] = (uint8_t) byte & 0xFF;
	}
	int read() {
		if (_rxAvailable > 0) {
			char byte = _rxBuffer[0];
			fifoBuffer::regShiftDown(_rxBuffer, _rxAvailable--);
			return byte;
		} else return -1;
	}
	int rxAvailable() {
		return _rxAvailable;
	}
	int txAvailable() {
		return _txAvailable;
	}
protected:
	void attachBuffers(char* txBuffer, char* rxBuffer) {
		_txBuffer = txBuffer;
		_txAvailable = 0;
		_rxBuffer = rxBuffer;
		_rxAvailable = 0;
	}
	void flushTx() {
		_txAvailable = 0;
	}
	void loadRxBuffer(char* buffer, int startIndex, int numberOfBytes) {
		for (int x = startIndex; x < numberOfBytes + startIndex; ++x)
			_rxBuffer[_rxAvailable++] = buffer[x];
	}
	char retrieveTxByte() {
		char byte = _txBuffer[0];
		fifoBuffer::regShiftDown(_txBuffer, _txAvailable--);
		return byte;
	}
private:
	void regShiftDown(char* buffer, int dataLength) {
		for (int x = 0; x < dataLength - 1; ++x)
			buffer[x] = buffer[x + 1];
	}
	char* _txBuffer;
	char* _rxBuffer;
	int _txAvailable;
	int _rxAvailable;
};

// Communication Objects
extern NightShade_Treo_TWI	Treo_TWI;
extern NightShade_Treo_GPIO	Treo_GPIO;
extern NightShade_Treo_SPI	Treo_SPI;
extern NightShade_Treo_UART	Treo_UART;

#endif /* NIGHTSHADE_TREO_DRIVER_H_ */
