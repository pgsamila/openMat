/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef CALC_MISALIGNMENT
#define CALC_MISALIGNMENT

#include "LpMatrix.h"

#ifdef __cplusplus
extern "C" {
#endif

float maCalPythag(float a, float b);
void maCalcalcSVD(float **mat, int m, int n, float **w, float **v, int maxCalElements);
void maCalCalcMisalignment(float **misalignmentAData, float **misalignmentBData, LpMatrix3x3f *R, LpVector3f *t, int nEquations);
void maCalCalcGyrMisalignment(float **misalignmentAData, float **misalignmentBData, LpMatrix3x3f *R, LpVector3f *t, int nEquations);
void maCalCalcMagMisalignment(float **misalignmentAData, float **misalignmentBData, LpMatrix3x3f *R, LpVector3f *t, int nEquations);

#ifdef __cplusplus
}
#endif 

#endif