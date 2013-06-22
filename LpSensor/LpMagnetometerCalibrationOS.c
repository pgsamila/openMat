/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpMagnetometerCalibration.h"

float bCalPythag(float a, float b)
{
	return 0.0f;
}

void bCalOrientationFromAccMag(LpVector3f b, LpVector3f a, LpVector3f *r, float *inc)
{
}

void bCalInitEllipsoidFit(void)
{
}

void bCalUpdateBMap(LpVector3f r, LpVector3f bRaw)
{
}

int bCalCalcSVD(float **mat, int m, int n, float **w, float **v, int maxCalElements)
{
	return 0;
}

int bCalFitEllipsoid(void)
{
	return 1;
}	

void bCalTestEllipsoidFit(void)
{
}

LpMatrix3x3f bCalGetSoftIronMatrix(void)
{
	LpMatrix3x3f m;
	
	return m;
}

LpVector3f bCalGetHardIronOffset(void)
{
	LpVector3f v;
	
	return v;
}

float bCalGetFieldRadius(void)
{
	return 0.0f;
}

void bCalSetSoftIronMatrix(LpMatrix3x3f m)
{
}

void bCalSetHardIronOffset(LpVector3f v)
{
}

void bCalSetFieldRadius(float r)
{
}

float bCalGetFieldMapElement(int i, int j, int k, int l)
{
	return 0.0f;
}

LpVector3f bCalCorrect(LpVector3f b)
{
	LpVector3f v;
	
	return v;
}