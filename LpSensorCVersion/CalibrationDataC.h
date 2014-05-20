/***********************************************************************
** Copyright (C) LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
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
***********************************************************************/

#ifndef CALIBRATION_DATA_C
#define CALIBRATION_DATA_C
              
#include <stdio.h>
#include <stdlib.h>
	
#include "../LpSensor/LpMatrix.h"
#include "../LpSensor/LpmsDefinitions.h"

typedef struct _CalibrationData
{
	char name[32];
	char deviceId[32];
	char firmwareVersion[32];
	
	float fieldRadius;
	
	int openMatId;
	int deviceType;
	int parameterSet;	
	int filterMode;
	int gyrThresEnable;	
	int quaternionCalcLocal;
	int samplingRate;	
	int gyrRange;
	int magRange;
	int accRange;
	int magAutocalibration;
	int canStreamFormat;
	int canBaudrate;
	int selfTestOn;	
	int magThreshold;
	int magOutOfRange;
	int gyrAutocalibration;
	int selectedData;
	int lowPassFilter;
	int canMapping[32];
	int canHeartbeat;
	int heavemotionEnabled;
	int gaitTrackingEnabled;
	int linAccCompMode;
	int centriCompMode;
	int canPointMode;
	int canChannelMode;
	int canStartId;
	int lpBusDataMode;
	int firmwareVersionDig0;
	int firmwareVersionDig1;
	int firmwareVersionDig2;
	
	LpVector3f magReference;
	LpVector3f magMAlignmentBias;
	LpVector3f gyrAlignmentBias;
	LpVector3f accBias;
	LpVector3f fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW];	
	LpVector3f hardIronOffset;
	
	LpMatrix3x3f softIronMatrix;
	LpMatrix3x3f misalignMatrix;
	LpMatrix3x3f gyrMisalignMatrix;
	LpMatrix3x3f magMAlignmentMatrix;
} CalibrationData;
	
void lpmsInitCalibrationData(CalibrationData* cd);

void lpmsSetParameterString(CalibrationData* cd, int parameterIndex, char* parameter);
void lpmsSetParameterInt(CalibrationData* cd, int parameterIndex, int parameter);
void lpmsSetParameterIntArray(CalibrationData* cd, int parameterIndex, int *parameter);	
void lpmsSetParameterFloat(CalibrationData* cd, int parameterIndex, float parameter);

void lpmsGetParameterString(CalibrationData* cd, int parameterIndex, char* parameter);
void lpmsGetParameterInt(CalibrationData* cd, int parameterIndex, int *parameter);
void lpmsGetParameterFloat(CalibrationData* cd, int parameterIndex, float *parameter);

void lpmsPrintParameters(CalibrationData* cd);

#endif
