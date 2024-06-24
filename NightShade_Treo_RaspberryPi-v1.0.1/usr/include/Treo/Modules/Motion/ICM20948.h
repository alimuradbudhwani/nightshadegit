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
 * ICM20948.h
 *
 *  Created on: Sep 17, 2019
 *      Author: Aaron D. Liebold
 */

#ifndef NIGHTSHADE_TREO_ICM20948_H_
#define NIGHTSHADE_TREO_ICM20948_H_

#include <Treo/Drivers/NightShade_Treo_Driver.h>

#define ICM20948_DEFAULT_SLAVEADDRESS 0x69
#define ICM20948_DEFAULT_CLOCKSPEED	400000u

#define GYRO_SENSITIVITY_DPS_0	131.0
#define GYRO_SENSITIVITY_DPS_1	65.5
#define GYRO_SENSITIVITY_DPS_2	32.8
#define GYRO_SENSITIVITY_DPS_3	16.4

#define ACCEL_SENSITIVITY_G_0	(1 << 14)
#define ACCEL_SENSITIVITY_G_1	(1 << 13)
#define ACCEL_SENSITIVITY_G_2	(1 << 12)
#define ACCEL_SENSITIVITY_G_3	(1 << 11)

#define TEMP_SENSITIVITY_HUNDRETH_DEGREE	3.3387

#define REG_BANK_SEL_ADDR	0x7F
#define REG_BANK_SEL_BANK0	(0 << 4)
#define REG_BANK_SEL_BANK1	(1 << 4)
#define REG_BANK_SEL_BANK2	(2 << 4)
#define REG_BANK_SEL_BANK3	(3 << 4)

/***	BANK 0 Registers	***/

// Operation Regs
#define WHO_AM_I		0x00
#define USER_CTRL		0x03
#define LP_CONFIG		0x05
#define PWR_MGMT_1		0x06
#define PWR_MGMT_2		0x07
#define INT_PIN_CFG		0x0F
#define INT_ENABLE		0x10
#define INT_ENABLE_1	0x11
#define INT_ENABLE_2	0x12
#define INT_ENABLE_3	0x13
#define I2C_MST_STATUS	0x17
#define INT_STATUS		0x19
#define INT_STATUS_1	0x1A
#define INT_STATUS_2	0x1B
#define INT_STATUS_3	0x1C

// Data Regs
#define ACCEL_XOUT_H	0x2D
#define ACCEL_XOUT_L	0x2E
#define ACCEL_YOUT_H	0x2F
#define ACCEL_YOUT_L	0x30
#define ACCEL_ZOUT_H	0x31
#define ACCEL_ZOUT_L	0x32
#define GYRO_XOUT_H		0x33
#define GYRO_XOUT_L		0x34
#define GYRO_YOUT_H		0x35
#define GYRO_YOUT_L		0x36
#define GYRO_ZOUT_H		0x37
#define GYRO_ZOUT_L		0x38
#define TEMP_OUT_H		0x39
#define TEMP_OUT_L		0x3A

// External Sensor Regs
#define EXT_SLV_SENS_DATA_00	0x3B
#define EXT_SLV_SENS_DATA_01	0x3C
#define EXT_SLV_SENS_DATA_02	0x3D
#define EXT_SLV_SENS_DATA_03	0x3E
#define EXT_SLV_SENS_DATA_04	0x3F
#define EXT_SLV_SENS_DATA_05	0x40
#define EXT_SLV_SENS_DATA_06	0x41
#define EXT_SLV_SENS_DATA_07	0x42
#define EXT_SLV_SENS_DATA_08	0x43
#define EXT_SLV_SENS_DATA_09	0x44
#define EXT_SLV_SENS_DATA_10	0x45
#define EXT_SLV_SENS_DATA_11	0x46
#define EXT_SLV_SENS_DATA_12	0x47
#define EXT_SLV_SENS_DATA_13	0x48
#define EXT_SLV_SENS_DATA_14	0x49
#define EXT_SLV_SENS_DATA_15	0x4A
#define EXT_SLV_SENS_DATA_16	0x4B
#define EXT_SLV_SENS_DATA_17	0x4C
#define EXT_SLV_SENS_DATA_18	0x4D
#define EXT_SLV_SENS_DATA_19	0x4E
#define EXT_SLV_SENS_DATA_20	0x4F
#define EXT_SLV_SENS_DATA_21	0x50
#define EXT_SLV_SENS_DATA_22	0x51
#define EXT_SLV_SENS_DATA_23	0x52

// FIFO Regs
#define FIFO_EN_1		0x66
#define FIFO_EN_2		0x67
#define FIFO_RST		0x68
#define FIFO_MODE		0x69
#define FIFO_COUNTH		0x70
#define FIFO_COUNTL		0x71
#define FIFO_R_W		0x72
#define DATA_RDY_STATUS	0x74
#define FIFO_CFG		0x76

/***	BANK 1 Registers	***/

// Bank Regs
#define SELF_TEST_X_GYRO	0x02
#define SELF_TEST_Y_GYRO	0x03
#define SELF_TEST_Z_GYRO	0x04
#define SELF_TEST_X_ACCEL	0x0E
#define SELF_TEST_Y_ACCEL	0x0F
#define SELF_TEST_Z_ACCEL	0x10
#define XA_OFFS_H			0x14
#define XA_OFFS_L			0x15
#define YA_OFFS_H			0x17
#define YA_OFFS_L			0x18
#define ZA_OFFS_H			0x1A
#define ZA_OFFS_L			0x1B
#define TIMEBASE_CORRECTION_PLL	0x28

/***	BANK 2 Registers	***/
#define GYRO_SMPLRT_DIV	0x00
#define GYRO_CONFIG_1	0x01
#define GYRO_CONFIG_2	0x02
#define XG_OFFS_USRH	0x03
#define XG_OFFS_USRL	0x04
#define YG_OFFS_USRH	0x05
#define YG_OFFS_USRL	0x06
#define ZG_OFFS_USRH	0x07
#define ZG_OFFS_USRL	0x08
#define ODR_ALIGN_EN	0x09
#define ACCEL_SMPLRT_DIV_1	0x10
#define ACCEL_SMPLRT_DIV_2	0x11
#define ACCEL_INTEL_CTRL	0x12
#define ACCEL_WOM_THR	0x13
#define	ACCEL_CONFIG	0x14
#define	ACCEL_CONFIG_2	0x15
#define FSYNC_CONFIG	0x52
#define	TEMP_CONFIG		0x53
#define MOD_CTRL_USR	0x54

/****	Bank 3 Registers	***/
#define I2C_MST_ODR_CONFIG	0x00
#define I2C_MST_CTRL		0x01
#define I2C_MST_DELAY_CTRL	0x02
#define I2C_SLV0_ADDR		0x03
#define I2C_SLV0_REG		0x04
#define I2C_SLV0_CTRL		0x05
#define I2C_SLV0_DO			0x06
#define I2C_SLV1_ADDR		0x07
#define I2C_SLV1_REG		0x08
#define I2C_SLV1_CTRL		0x09
#define I2C_SLV1_DO			0x0A
#define I2C_SLV2_ADDR		0x0B
#define I2C_SLV2_REG		0x0C
#define I2C_SLV2_CTRL		0x0D
#define I2C_SLV2_DO			0x0E
#define I2C_SLV3_ADDR		0x0F
#define I2C_SLV3_REG		0x10
#define I2C_SLV3_CTRL		0x11
#define I2C_SLV3_DO			0x12
#define I2C_SLV4_ADDR		0x13
#define I2C_SLV4_REG		0x14
#define I2C_SLV4_CTRL		0x15
#define I2C_SLV4_DO			0x16
#define I2C_SLV4_DI			0x17

/**	Magnetometer Registers	***/
#define MAG_SLAVE_ADDR	0x0C
#define MAG_WIA2	0x01
#define MAG_ST1		0x10
#define MAG_HXL		0x11
#define MAG_HXH		0x12
#define MAG_HYL		0x13
#define MAG_HYH		0x14
#define MAG_HZL		0x15
#define MAG_HZH		0x16
#define MAG_ST2		0x18
#define MAG_CNTL2	0x31
#define MAG_CNTL3	0x32

class NightShade_Treo_ICM20948 {
public:
	NightShade_Treo_ICM20948(int port, uint8_t slaveAddress,
			uint32_t clockSpeed);
	NightShade_Treo_ICM20948(int port);
	int begin();

	uint8_t readDeviceId();
	int enableSleep(int enable);
	int clockSelect(uint8_t clkSource);
	int enableSensors(int enableAccel, int enableGyro, int enableTemp);
	int enableLowPower(int enable);
	int setupSensors(uint8_t accelSensitivity, uint8_t accelFilter,
			uint8_t gyroSensitivity, uint8_t gyroFilter);
	int enableI2cPassthrough(int enable);
	uint8_t readIntPinCfg();

	// Read data
	int getData(int getAccelGyroTempData, int getCompassData);
	int readAccelX();
	int readAccelY();
	int readAccelZ();
	int readGyroX();
	int readGyroY();
	int readGyroZ();
	int readCompassX();
	int readCompassY();
	int readCompassZ();
	int readTemperature();

	float accelSensitivity();
	float gyroSensitivity();

	uint8_t magReadDeviceId();
	int magDataReady();
	int magBegin();
	int magReadData();
	int magSetMeasurementMode(uint8_t mode);
	int magReset(uint8_t enable);

private:
	int _port;
	uint8_t _slaveAddress;
	uint32_t _clockSpeed;

	int _accelDataX;
	int _accelDataY;
	int _accelDataZ;
	float _accelSensitivity;

	int _gyroDataX;
	int _gyroDataY;
	int _gyroDataZ;
	float _gyroSensitivity;

	int _tempData;

	int _compassDataX;
	int _compassDataY;
	int _compassDataZ;

	int setRegisterBank(int bankNumber);
	uint8_t _currentRegBank;
};

#endif /* NIGHTSHADE_TREO_ICM20948_H_ */
