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

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "LpMatrix.h"

#ifdef __cplusplus
extern "C" {
#endif 

// Calculates phythagorean distance
float bCalPythag(float a, float b);

// Calculates current orientation of sensor from acceleration and e-compass
void bCalOrientationFromAccMag(LpVector3f b, LpVector3f a, LpVector3f *r, float *inc);

// Initializes ellipsoid fitting algorithm
void bCalInitEllipsoidFit(void);

// Updates the magnetic field map with current raw magnetometer data
void bCalUpdateBMap(LpVector3f r, LpVector3f bRaw);

// Calculates singular vector decomposition
int bCalCalcSVD(float **mat, int m, int n, float **w, float **v, int maxCalElements);

// Fits ellpsoid to current field map
int bCalFitEllipsoid(void);

// Tests ellipsoid fitting algorithm
void bCalTestEllipsoidFit(void);

// Retrieves current soft iron matrix
LpMatrix3x3f bCalGetSoftIronMatrix(void);

// Retrieves hard iron offset vector
LpVector3f bCalGetHardIronOffset(void);

// Retrieves current absolute field strength
float bCalGetFieldRadius(void);

// Sets current soft iron matrix
void bCalSetSoftIronMatrix(LpMatrix3x3f m);

// Sets current ard iron offset
void bCalSetHardIronOffset(LpVector3f v);

// Sets current absolute field strength value
void bCalSetFieldRadius(float r);

// Corrects raw magnetometer data
LpVector3f bCalCorrect(LpVector3f b);

// Retrieves a field element at a specified location
float bCalGetFieldMapElement(int i, int j, int k, int l);

void getReferenceYZ(LpVector3f b, LpVector3f a, LpVector3f *r, float *inc);

#ifdef __cplusplus
}
#endif 

#endif