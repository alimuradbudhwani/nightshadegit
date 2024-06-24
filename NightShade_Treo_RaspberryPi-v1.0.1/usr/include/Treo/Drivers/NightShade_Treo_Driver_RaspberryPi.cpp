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
 * NightShade_Treo_RaspberryPi_Driver.cpp
 *
 *  Created on: May 13, 2019
 *      Author: Aaron D. Liebold
 */

#include <stdio.h>
#include <stdint.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>
#include "NightShade_Treo_Driver.h"

// Communication Objects
NightShade_Treo_TWI		Treo_TWI;
NightShade_Treo_GPIO	Treo_GPIO;
NightShade_Treo_SPI		Treo_SPI;
NightShade_Treo_UART	Treo_UART;

// ********* General Functions *********** //

void Treo_Delay(unsigned long delay_ms) {
	delay(delay_ms);
//	usleep(delay_ms * 1000);
}

void Treo_Delayus(unsigned long delay_us) {
	delayMicroseconds(delay_us);
//	usleep(delay_us);
}

// *************************************** //

// **************** TWI ****************** //
NightShade_Treo_TWI::NightShade_Treo_TWI() {
	_port = 0;
	_device = 0;
	_slaveAddress = 0;
	_clockSpeed = 0;
}

int NightShade_Treo_TWI::begin(int port, uint8_t sevenBitAddress, uint32_t clockSpeed) {
	_port = port;
	_clockSpeed = clockSpeed;
	char *i2cDevice = (char *) "/dev/i2c-0";
	if (port == 1) i2cDevice = (char*) "/dev/i2c-1";

	if ((_device = open(i2cDevice, O_RDWR)) < 0) {
		// printf("Failed to open I2C.\n");
		return 1;
	}
	if (ioctl(_device, I2C_SLAVE, sevenBitAddress) < 0) {
		// printf("Failed to attach I2C.\n");
		return 2;
	}
	return 0;
}
int NightShade_Treo_TWI::write(uint8_t *buffer, int numberOfBytes, int sendStop) {
	if (::write(_device, buffer, numberOfBytes) != numberOfBytes) {
		// printf("Failed to write I2C.");
		return -1;
	} else return 0;
}
int NightShade_Treo_TWI::read(uint8_t *buffer, int numberOfBytes, int sendStop) {
	if (::read(_device, buffer, numberOfBytes) != numberOfBytes) {
		// printf("Failed to read I2C.");
		return -1;
	} else return 0;
}
int NightShade_Treo_TWI::end() {
	if (close(_device) < 0) {
		// printf("Failed to close I2C.");
		return -1;
	} else return 0;
}
// *************** end TWI **************** //

// **************** GPIO ****************** //
NightShade_Treo_GPIO::NightShade_Treo_GPIO() {

}

void NightShade_Treo_GPIO::setup(int pin, int mode) {
	wiringPiSetupGpio();
	pinMode(pin, mode);
}
void NightShade_Treo_GPIO::write(int pin, int state) {
	if (state == GPIO_HIGH) {
		digitalWrite(pin, 1); // Set pin state high
	} else {
		digitalWrite(pin, 0); // Set pin state low
	}
}
int NightShade_Treo_GPIO::read(int pin) {
	return digitalRead(pin);
}
// ************** end GPIO **************** //

// **************** SPI ****************** //
NightShade_Treo_SPI::NightShade_Treo_SPI() {
	_device = 0;
	_port = 0;
	_csPin = 0;
	_spiMode = 0;
	_bitsPerWord = 0;
	_clockSpeed = 0;
	_lsbFirst = 0;
}

int NightShade_Treo_SPI::begin(int port, int chipSelectPin, uint32_t baudrate, int spiMode,
		int bitsPerWord, int lsbFirst) {
	uint8_t mode = 0, ret;

	char *spiDevice = (char *) "/dev/spidev0.0";
	if (_port == 1) {
		spiDevice = (char *) "/dev/spidev1.0";
	}

	_port = port;
	_csPin = chipSelectPin;
	_spiMode = spiMode;
	_bitsPerWord = bitsPerWord;
	_clockSpeed = baudrate;

	switch (spiMode) {
	case 0:
		_spiMode = SPI_MODE_0;
		break;
	case 1:
		_spiMode = SPI_MODE_1;
		break;
	case 2:
		_spiMode = SPI_MODE_2;
		break;
	case 3:
		_spiMode = SPI_MODE_3;
		break;
	default:
		_spiMode = SPI_MODE_0;
		break;
	}

	// Set no Chip Select mode
	mode |= SPI_NO_CS;

	// Set LSB first
	if (lsbFirst) mode |= SPI_LSB_FIRST;

	_device = open(spiDevice, O_RDWR);
	if (_device < 0) {
		// printf("SPI: Failed to open SPI port.\n");
		return 1;
	}

	ret = ioctl(_device, SPI_IOC_WR_MODE, &_spiMode);
	if (ret == -1) {
		// printf("SPI: Failed to setup SPI.\n");
		return 1;
	}

	ret = ioctl(_device, SPI_IOC_WR_BITS_PER_WORD, &_bitsPerWord);
	if (ret == -1) {
		// printf("SPI: Failed to set bitsPerWord.\n");
		return 3;
	}

	ret = ioctl(_device, SPI_IOC_WR_MAX_SPEED_HZ, &_clockSpeed);
	if (ret == -1) {
		// printf("SPI: Failed to set baudrate.\n");
		return 4;
	}

	// Setup CS pin
	Treo_GPIO.setup(_csPin, GPIO_OUTPUT);
	Treo_GPIO.write(_csPin, GPIO_HIGH);

	return 0;
}
int NightShade_Treo_SPI::transfer(uint8_t *buffer, int numberOfBytes) {
	int ret = -1;
	struct spi_ioc_transfer tr = { 0 };

	tr.tx_buf = (unsigned long) (buffer);
	tr.rx_buf = (unsigned long) (buffer);
	tr.len = numberOfBytes;
	tr.delay_usecs = 0;
	tr.speed_hz = _clockSpeed;
	tr.bits_per_word = _bitsPerWord;
	tr.cs_change = 0;

	digitalWrite(_csPin, LOW);
	ret = ioctl(_device, SPI_IOC_MESSAGE(1), &tr);
	digitalWrite(_csPin, HIGH);

	// if (ret < 1) printf("Transfer Error!!! First Byte Was: 0x%x\n", buffer[0]);

	return ret;
}
int NightShade_Treo_SPI::end() {
	if (close(_device)) {
		// printf("SPI: Failed to close port.\n");
		return -1;
	}
	return 0;
}
// ************** end SPI **************** //

// **************** UART ****************** //

// Pairity: 0 - None, 1 - Odd, 2 - Even
// Databits: 5-8
// End bit(s): 1-2
NightShade_Treo_UART::NightShade_Treo_UART(){
	_port = 0;
	_device = 0;
	_baudrate = 0;
	_parity = 0;
	_dataBits = 0;
	_endBits = 0;
}

void NightShade_Treo_UART::begin(int port, uint32_t baudrate, uint8_t dataBits,
		uint8_t pairity, uint8_t endBits) {

}
int NightShade_Treo_UART::write(uint8_t *buffer, int numberOfBytes) {
	return 0;
}
int NightShade_Treo_UART::read(uint8_t *buffer, int numberOfBytes) {
	return 0;
}

int NightShade_Treo_UART::end() {
	return 0;
}
// **************** end UART ****************** //

