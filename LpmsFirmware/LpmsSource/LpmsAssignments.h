/***********************************************************************
** Functions for data assignments and conversions
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_ASSIGNMENTS_H
#define LPMS_ASSIGNMENTS_H

#include "stm32f2xx.h"     

#include "LpMatrix.h"
#include "LpmsConfig.h"
#include "LpmsL3gd20.h"
#include "LpmsBmp180.h"
#include "LpmsLsm303dlhc.h"
#include "LpFilterCVersion.h"
#include "LpmsTimebase.h"
#include "LpMagnetometerCalibration.h"
#include "LpmsTest.h"

// Number format indicators
#define FLOAT_FULL_PRECISION 0
#define FLOAT_HALF_PRECISION 1
#define FLOAT_FIXED_POINT_10 2
#define FLOAT_FIXED_POINT_100 3
#define FLOAT_FIXED_POINT_1000 4

// Sets register
void setReg(uint16_t length, uint8_t* data, uint32_t address);

// Retrieves register
void getReg(uint8_t* data, uint16_t length, uint32_t address);

// Converts IEEE floating point to integer format
uint32_t conFtoI(float f);

// Converts integer format to IEEE floating point
float conItoF(uint32_t v);

// Retrieves 32-bit unsigned integer from communication data
uint32_t getUi32t(uint8_t* data);

// Retrives IEEE floating point from communication data
float getFloat(uint8_t* data);

// Sets 32-bit unsigned integer to communication data
void setUi32t(uint8_t* data, uint32_t v);

// Sets 32-bit float to communication data
void setFloat(uint8_t* data, float f, uint8_t prec);

// Sets 32-bit unsigned integer array to communication data 
void setMultiUi32t(uint8_t* data, uint32_t *v, int l);

// Retrieves LpVector3f from communication data
LpVector3f getVector3f(uint8_t* data);

// Retrieves LpVector4f from communication data
LpVector4f getVector4f(uint8_t* data);

// Sets register to float value
uint8_t setRegFloat(uint8_t i, float v);

// Sets register to 32-bit unsigned integer
uint8_t setRegUInt32(uint8_t i, uint32_t v);

// Sets register to LpVector3f
uint8_t setRegVector3f(uint8_t i, LpVector3f v);

// Sets register to LpVector4f
uint8_t setRegVector4f(uint8_t i, LpVector4f v);

// Sets register to LpMatrix3x3f
uint8_t setRegMatrix3x3f(uint8_t i, LpMatrix3x3f m);

// Writes register to flash memory
uint8_t writeRegToFlash(uint8_t i);

// Sets 8-bit unsigned integer to communication data
void setUi8t(uint8_t* data, uint8_t v);

// Retrives 32-bit unsigned integer array from communication data 
void getMultiUi32t(uint8_t* data, uint8_t n, uint32_t* v);

// Sets LpVector3f to communication data
void setFloatVector3f(uint8_t* data, int o, LpVector3f f);

// Sets LpMatrix3x3f to communication data
void setFloatMatrix3x3f(uint8_t* data, int o, LpMatrix3x3f M);

// Sets big endian float to communication data
void setFloatBigEndian(uint8_t* data, float f);

// Retrieves LpMatrix3x3f from communication data
LpMatrix3x3f getMatrix3x3f(uint8_t* data);

void setI16t(uint8_t* data, int16_t v);

#endif