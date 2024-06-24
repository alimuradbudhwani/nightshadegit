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
 * NightShade_Treo.h
 *
 *  Created on: Apr 16, 2020
 *      Author: Aaron D. Liebold
 */

#ifndef NIGHTSHADE_TREO_H_
#define NIGHTSHADE_TREO_H_

// Driver
#include <Treo/Drivers/NightShade_Treo_Driver.h>

// Analog
#include <Treo/Modules/Analog/ACS70331.h>
#include <Treo/Modules/Analog/ACS711.h>
#include <Treo/Modules/Analog/MAX11612.h>
#include <Treo/Modules/Analog/MAX11644.h>

// Environmental
#include <Treo/Modules/Environmental/HDC1080.h>
#include <Treo/Modules/Environmental/HX711.h>
#include <Treo/Modules/Environmental/LIS3MDL.h>
#include <Treo/Modules/Environmental/MCP9600.h>
#include <Treo/Modules/Environmental/OPT3002.h>

// Interfaces
#include <Treo/Modules/Interfaces/DigitalInput.h>
#include <Treo/Modules/Interfaces/DigitalOutput.h>
#include <Treo/Modules/Interfaces/HT16K33.h>
#include <Treo/Modules/Interfaces/NumberPad.h>

// IO Expanders
#include <Treo/Modules/IOExpanders/TCA9534A.h>

// Memory
#include <Treo/Modules/Memory/CAT25512.h>

// Motion
#include <Treo/Modules/Motion/ADXL345.h>
#include <Treo/Modules/Motion/ICM20948.h>

#endif /* NIGHTSHADE_TREO_H_ */
