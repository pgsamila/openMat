/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpFilterCVersion.h"

void lpFilterInit(void)
{
}

/* 	INPUT
	a: Calib. accelerometer data (a.data[0] = x-axis, a.data[1] = y-axis etc.)
	b: Calib. magnetometer data
	g: Calib. gyroscope data
	T: Timestamp (Time difference between frames in ms)
	
	OUTPUT:
	qOut: Orientation quaternion (q.data[0] = w-axis, q.data[1] = x-axis etc.) */
void lpFilterUpdate(LpVector3f a, LpVector3f b, LpVector3f g, LpVector4f *qOut,
	float T, float bInclination, float *bNoise)
{	
}

void lpOrientationFromAccMag(LpVector3f b, LpVector3f a, LpVector3f *r, float *i)
{
}

void lpFilterEulerUpdate(LpVector3f a, LpVector3f b, LpVector3f g, LpVector3f *rOut, LpVector4f *qOut,
	float T, float bInclination, float *bNoise)
{	
}

void gyrOnlineCal(float T)
{
}

void gyroToInertial(LpVector4f qI, LpVector3f *wO)
{
}

void gyroToInertialEuler(void)
{
}