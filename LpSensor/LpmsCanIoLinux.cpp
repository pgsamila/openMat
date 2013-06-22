/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsCanIo.h"

LpmsCanIo::LpmsCanIo(CalibrationData *configData) :
	LpmsIoInterface(configData)
{
}

LpmsCanIo::~LpmsCanIo(void)
{
}

bool LpmsCanIo::connect(std::string deviceId) 
{
	(void) deviceId;

	return true;
}

bool LpmsCanIo::sendModbusData(unsigned address, unsigned function, 
	unsigned length, unsigned char *data)	
{
	(void) address;
	(void) function;
	(void) length;
	(void) data;

	return true;
}

bool LpmsCanIo::parseModbusByte(unsigned char b)
{	
	(void) b;

	return true;
}
