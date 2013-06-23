/***********************************************************************
** Magnetometer data correction and reference vector adjustment
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
**
** Note: Parts of this code have been adapted from Freescale 
** application note AN4248. Main calibration code in LpSensor
** has been completely changed to proprietary code in the meantime.
***********************************************************************/

#ifndef LP_MAGNETOMETER_CALIBRATION
#define LP_MAGNETOMETER_CALIBRATION

#ifndef __IAR_SYSTEMS_ICC__
	#include <stdio.h> 
	#include <conio.h>
#endif

#include <math.h>
#include <stdlib.h> 
#include <time.h> 

#include "LpMatrix.h" 

#ifdef __cplusplus
extern "C" {
#endif 

// Degree to radians conversion
#define DegToRad 0.0174532925199433f
#define RadToDeg 57.2957795130823f

// Applies low-pass filter
void fModuloLPF(float Angle, float *pLPFAngle);

// Applies hard and soft-iron scaling
void fInvertHardandSoftIron(void);

// Initializes magnetometer calibration
void initializeMCal(void);

// Corrects current raw magnetometer sensor data
void correctB(float *bX, float *bY, float *bZ);

// Retrieves soft iron matrix
void getSoftIronMatrix(LpMatrix3x3f *m);

// Retrievs hard iron offset
void getHardIronOffset(LpVector3f *o);

// Sets soft iron matrix
void setSoftIronMatrix(LpMatrix3x3f m);

// Sets hard iron ofset
void setHardIronOffset(LpVector3f o);

// Retrieves reference Y and Z axis
void getReferenceYZ(float fBx, float fBy, float fBz, 
	float fGx, float fGy, float fGz,
	float *refY, float *refZ,
	float *bInc);


void feCompass(float fBx, float fBy, float fBz, 
	float fGx, float fGy, float fGz,
	float *bInc,
	float *phiOut, float *thetaOut, float *psiOut);

#ifdef __cplusplus
}
#endif 
	
#endif