/***********************************************************************
** Sensor self-test and input simulation 
**
** Copyright (C) 2013 LP-RESEARCH Inc.
** All rights reserved.
** Contact: info@lp-research.com
***********************************************************************/

#ifndef LPMS_TEST_H
#define LPMS_TEST_H

#include "LpMatrix.h"

// Contains parameters for testing data processing (Kalman filter etc.).
typedef struct _FilterTestParameters {	
	LpVector3f magOff;
	LpVector3f magDis;

	LpVector3f gravity;
	LpVector3f magField;

	LpVector3f gyrError;
} FilterTestParameters;

// Fills the local test parameter structure.
void setTestParameters(LpVector3f magOff,
	LpVector3f magDis,
	LpVector3f gravity,
	LpVector3f magField,
	LpVector3f gyrError);

// Generates simulated gyroscope, accelerometer and magnetometer data.
void horizontalRotationNext(LpVector3f *g, LpVector3f *a, LpVector3f *b, 
	LpVector3f gyrGain, LpVector3f accGain, LpVector3f magGain,
	LpVector3f aRef, LpVector3f bRef, 
	float t);

#endif