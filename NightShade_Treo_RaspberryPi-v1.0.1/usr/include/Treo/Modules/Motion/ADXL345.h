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
 * ADXL345.h
 *
 *  Created on: Jun 20, 2019
 *      Author: Aaron D. Liebold
 */

#ifndef NIGHTSHADE_TREO_ADXL345_H_
#define NIGHTSHADE_TREO_ADXL345_H_

#include <Treo/Drivers/NightShade_Treo_Driver.h>

class NightShade_Treo_ADXL345 {
public:
	// Constructors and begin
	NightShade_Treo_ADXL345(int port);
	NightShade_Treo_ADXL345(int port, uint8_t slaveAddress, uint32_t clockSpeed);
	int begin();

	// Read data
	int retrieveData();
	int readX();
	int readY();
	int readZ();

	// Power Settings
	int enableLink(uint8_t enabled);
	int enableAutoSleep(uint8_t enable);
	int enableMeasureMode(uint8_t enable);
	int enableSleep(uint8_t enable);
	int setWakeUp(uint8_t mode);
	int enableLowPower(uint8_t enable);
	int setBandwidth(uint8_t setting);

	// Data Format Settings
	int enableSelfTest(uint8_t enable);
	int setOffsetValues(uint8_t xOffest, uint8_t yOffest, uint8_t zOffest);
	int invertInterruptOutput(uint8_t enable);
	int enableFullResolution(uint8_t enable);
	int setMeasurementRange(uint8_t mode);

	// FIFO Functions
	int setFifoMode(uint8_t mode);
	int enableFifoTrigger(uint8_t mode);
	int setFifoSamples(uint8_t numberOfSamples);
	int numberFifoEntries();
	int checkFifoTrigger();

	// Interrupt Functions
	int enableInterrupts(uint8_t regValue);
	int writeInterruptMap(uint8_t regValue);
	int readInterruptSources();

	// Tap Functions
	int setTapThreshold(uint8_t threshold);
	int setTapDuration(uint8_t duration);
	int setTapLatency(uint8_t latencyTime);
	int setDoubleTapWindow(uint8_t windowTime);

	// Acitvity / Inactivity Settings
	int setActivityThreshold(uint8_t threshold);
	int setInactivitiyThreshold(uint8_t threshold);
	int setInactivityTime(uint8_t time);
	int setActInactControl(uint8_t regValue);
	int setTapAxes(uint8_t regValue);
	int readActivityTapStatus();

	// Free-Fall (FF) Settings
	int setFFThreshold(uint8_t threshold);
	int setFFTime(uint8_t time);


private:
	int leftJustifyData(uint8_t enable);

	uint8_t _xDataAvailable = 0;
	uint8_t _yDataAvailable = 0;
	uint8_t _zDataAvailable = 0;
	int _xData[33] = { 0 };
	int _yData[33] = { 0 };
	int _zData[33] = { 0 };
	uint8_t _port;
	uint8_t _slaveAddress;
	uint32_t _clockSpeed;

	// Private Functions
	void regShiftDown(int* buffer, int dataLength);
};

#endif /* NIGHTSHADE_TREO_ADXL345_H_ */
