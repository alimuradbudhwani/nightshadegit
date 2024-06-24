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
 * LIS3MDL.h
 *
 *  Created on: Mar 25, 2020
 *      Author: Aaron D. Liebold
 */

#ifndef LIS3MDL_H_
#define LIS3MDL_H_

#include <Treo/Drivers/NightShade_Treo_Driver.h>

#define LIS3MDL_SLAVE_ADDR 			0x1E
#define LIS3MDL_DEFAULT_CLOCK_SPEED	400000


#define LIS3MDL_REG_WHO_AM_I	0x0F
#define LIS3MDL_REG_CTRL_REG1	0x20
#define LIS3MDL_REG_CTRL_REG2	0x21
#define LIS3MDL_REG_CTRL_REG3	0x22
#define LIS3MDL_REG_CTRL_REG4	0x23
#define LIS3MDL_REG_CTRL_REG5	0x24
#define LIS3MDL_REG_STATUS_REG	0x27
#define LIS3MDL_REG_OUT_X_L		0x28
#define LIS3MDL_REG_OUT_X_H		0x29
#define LIS3MDL_REG_OUT_Y_L		0x2A
#define LIS3MDL_REG_OUT_Y_H		0x2B
#define LIS3MDL_REG_OUT_Z_L		0x2C
#define LIS3MDL_REG_OUT_Z_H		0x2D
#define LIS3MDL_REG_TEMP_OUT_L	0x2E
#define LIS3MDL_REG_TEMP_OUT_H	0x2F
#define LIS3MDL_REG_INT_CFG		0x30
#define LIS3MDL_REG_INT_SRC		0x31
#define LIS3MDL_REG_INT_THS_L	0x32
#define LIS3MDL_REG_INT_THS_H	0x33


class NightShade_Treo_LIS3MDL {
public:
	NightShade_Treo_LIS3MDL(int port, uint8_t slaveAddress, uint32_t clockSpeed);
	NightShade_Treo_LIS3MDL(int port);

	// Setup Functions
	int begin();
	int	setOutputDataRate(int setting);
	int	setOperatingMode(int xyMode, int zMode);
	int	setMeasurementMode(int setting);
	int	enableTemperature(int enable);
	int	setFullScaleRange(int setting);
	int	enableInterrupt(int enableIntX, int enableIntY, int enableIntZ);
	int	setInterruptThreshold(int threshold);
	uint8_t readInterruptFlags();

	// Data Functions
	int acquireMagData();
	int readX();
	int readY();
	int readZ();
	int readTemp();

	// Utility Functions
	uint8_t	deviceId();
	int	enableSelfTest(int enable);
	int	oneGaussValue();
	int	dataReady();
	int	rebootMemory();
	int	restart();


private:
	int _port;
	uint8_t _slaveAddr;
	uint32_t _clockSpeed;
	int	_data[4];
	int	_operatingMode;
	int _oneGaussValue;
	int _measurementMode;
};



#endif /* LIS3MDL_H_ */
