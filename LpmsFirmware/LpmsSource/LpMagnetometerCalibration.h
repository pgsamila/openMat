/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** Redistribution and use in source and binary forms, with 
** or without modification, are permitted provided that the 
** following conditions are met:
**
** Redistributions of source code must retain the above copyright 
** notice, this list of conditions and the following disclaimer.
** Redistributions in binary form must reproduce the above copyright 
** notice, this list of conditions and the following disclaimer in 
** the documentation and/or other materials provided with the 
** distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
** FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
** NOTE: Initially parts of this code have been adapted from Freescale 
** application note AN4248.
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

// Initializes soft and hard-iron data
void initializeMCal(void);

// Corrects raw magnetic field data
LpVector3f correctB(LpVector3f b);
	
// Retrieves field direction
float getFieldDirection(void);

// Retrieves soft iron matrix
LpMatrix3x3f getSoftIronMatrix(void);

// Retrives hard iron offset vector
LpVector3f getHardIronOffset(void);

// Retrieves soft iron matrix
void setSoftIronMatrix(LpMatrix3x3f m);

// Sets hard iron offset vector
void setHardIronOffset(LpVector3f v);

// Retrieves magnetic reference vector
void getReferenceYZ(LpVector3f b, LpVector3f a, LpVector3f *r, float *inc);

// Calculates orientation from accelerometer and magnetometer
void bCalOrientationFromAccMag(LpVector3f b, LpVector3f a, LpVector3f *r, float *inc);

// Modulo low-pass filters Euler angle
void modLpFilter(float a, float *lpA);

#ifdef __cplusplus
}
#endif 
	
#endif