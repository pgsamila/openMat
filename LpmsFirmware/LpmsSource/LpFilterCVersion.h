/***********************************************************************
** Filter for sensor fusion 
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LP_FILTER_C_VERSION
#define LP_FILTER_C_VERSION

#include "LpMatrix.h"
#include "LpmsFactorySetting.h"

#ifdef __cplusplus
extern "C" {
#endif 

void lpFilterInit(void);
void lpFilterUpdate(LpVector3f a, LpVector3f b, LpVector3f g, LpVector4f *qOut, float T, float bInclination, float *bNoise);
void lpFilterEulerUpdate(LpVector3f a, LpVector3f b, LpVector3f g, LpVector3f *rOut, LpVector4f *qOut, float T, float bInclination, float *bNoise);
void lpOrientationFromAccMag(LpVector3f b, LpVector3f a, LpVector3f *r, float *i);
void setReferences(void);
void setCovariances(void);
void gyrOnlineCal(float T);
void gyroToInertial(LpVector4f qI, LpVector3f *wO);
void gyroToInertialEuler(void);
LpVector4f applyAlignmentOffset(LpVector4f q, uint32_t mode);
void calculateAlignmentOffset(LpVector4f q);

#ifdef __cplusplus
}
#endif 
	
#endif