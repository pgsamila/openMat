/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LP_MAGNETOMETER_CALIBRATION
#define LP_MAGNETOMETER_CALIBRATION

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "LpMatrix.h"

#ifdef __cplusplus
extern "C" {
#endif 

float bCalPythag(float a, float b);
void bCalOrientationFromAccMag(LpVector3f b, LpVector3f a, LpVector3f *r, float *inc);
void bCalInitEllipsoidFit(void);
void bCalUpdateBMap(LpVector3f r, LpVector3f bRaw);
int bCalCalcSVD(float **mat, int m, int n, float **w, float **v, int maxCalElements);
int bCalFitEllipsoid(void);
void bCalTestEllipsoidFit(void);
LpMatrix3x3f bCalGetSoftIronMatrix(void);
LpVector3f bCalGetHardIronOffset(void);
float bCalGetFieldRadius(void);
void bCalSetSoftIronMatrix(LpMatrix3x3f m);
void bCalSetHardIronOffset(LpVector3f v);
void bCalSetFieldRadius(float r);
LpVector3f bCalCorrect(LpVector3f b);
float bCalGetFieldMapElement(int i, int j, int k, int l);

#ifdef __cplusplus
}
#endif 

#endif