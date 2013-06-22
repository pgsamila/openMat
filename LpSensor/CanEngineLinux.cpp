/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "CanEngine.h"

CanEngine::CanEngine(void) 
{
}

CanEngine::~CanEngine(void) 
{
}

void CanEngine::listDevices(LpmsDeviceList *v)
{	
	(void) v;
}

void CanEngine::connect(void) 
{
}

void CanEngine::addSensor(LpmsCanIo *s)
{
	(void) s;
}

void CanEngine::removeSensor(LpmsCanIo *s)
{
	(void) s;
}

void CanEngine::poll() {
}

void CanEngine::close() {
}

bool CanEngine::updateSendQueue(void)
{	
	return true;
}
