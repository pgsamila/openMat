/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsTest.h"

#include <stdio.h>
#include <stdlib.h>

static float d2r = 0.01745f;
static float r = 0.0f;
static float rPrev = 0.0f;
static int direction = 0;
static int mode = 0;
static float w = 0.0f;
FilterTestParameters testPrm;

void setTestParameters(LpVector3f magOff,
	LpVector3f magDis,
	LpVector3f gravity,
	LpVector3f magField,
	LpVector3f gyrError) {
	testPrm.magOff = magOff;
	testPrm.magDis = magDis;
	testPrm.gravity = gravity;
	testPrm.magField = magField;
	testPrm.gyrError = gyrError;	
}

void horizontalRotationNext(LpVector3f *g, LpVector3f *a, LpVector3f *b, 
	LpVector3f gyrGain, LpVector3f accGain, LpVector3f magGain,
	LpVector3f aRef, LpVector3f bRef,
	float t)
{
	float fAmp = 40.0f;
	float xOff = 20.0f;
	float yOff = -70.0f;
	float zOff = -90.0f;
	float xDis = 1.0f;
	float yDis = 1.2f;
	float zDis = 1.5f;

	uint16_t xErr = (rand() % 500) - 30;
	uint16_t yErr = (rand() % 500) - 30;
	uint16_t zErr = (rand() % 500) - 30;

	LpVector3f aF;
	LpVector3f bF;

	LpMatrix3x3f rX;
	LpMatrix3x3f rY;
	LpMatrix3x3f rZ;

	aRef.data[0] = 0.0f;
	aRef.data[1] = 0.0f;
	aRef.data[2] = -1.0f;

	bRef.data[0] = 0.0f;
	bRef.data[1] = 1.9f;
	bRef.data[2] = -1.6f;

	rX.data[0][0] = 1;
	rX.data[0][1] = 0;
	rX.data[0][2] = 0;
	rX.data[1][0] = 0;
	rX.data[1][1] = cos(r * d2r);
	rX.data[1][2] = -sin(r * d2r);
	rX.data[2][0] = 0;
	rX.data[2][1] = sin(r * d2r);
	rX.data[2][2] = cos(r * d2r);

	rY.data[0][0] = cos(r * d2r);
	rY.data[0][1] = 0;
	rY.data[0][2] = sin(r * d2r);
	rY.data[1][0] = 0;
	rY.data[1][1] = 1;
	rY.data[1][2] = 0;
	rY.data[2][0] = -sin(r * d2r);
	rY.data[2][1] = 0;
	rY.data[2][2] = cos(r * d2r);

	rZ.data[0][0] = cos(r * d2r);
	rZ.data[0][1] = -sin(r * d2r);
	rZ.data[0][2] = 0;
	rZ.data[1][0] = sin(r * d2r);
	rZ.data[1][1] = cos(r * d2r);
	rZ.data[1][2] = 0;
	rZ.data[2][0] = 0;
	rZ.data[2][1] = 0;
	rZ.data[2][2] = 1;

	g->data[0] = 0.0f;
	g->data[1] = 0.0f;	
	g->data[2] = 0.0f;	

	switch (mode) {
	case 0:
		matVectMult3(&rZ, &aRef, &aF);
		matVectMult3(&rZ, &bRef, &bF);
		if (t > 0.0f) g->data[2] = -(r-rPrev) / t / gyrGain.data[2];
		rPrev = r;
	break;

	case 1:
		matVectMult3(&rX, &aRef, &aF);
		matVectMult3(&rX, &bRef, &bF);
		if (t > 0.0f) g->data[0] = -(r-rPrev) / t / gyrGain.data[0];
		rPrev = r;
	break;

	case 2:
		matVectMult3(&rY, &aRef, &aF);
		matVectMult3(&rY, &bRef, &bF);
		if (t > 0.0f) g->data[1] = -(r-rPrev) / t / gyrGain.data[1];
		rPrev = r;
	break;
	}

	g->data[0] += xErr;
	g->data[1] += yErr;
	g->data[2] += zErr;

	a->data[0] = (int16_t) (aF.data[0] / accGain.data[0]);
	a->data[1] = (int16_t) (aF.data[1] / accGain.data[1]);
	a->data[2] = (int16_t) (aF.data[2] / accGain.data[2]);

	scalarVectMult3x1(fAmp, &bF, &bF);
	bF.data[0] = (bF.data[0] * xDis) + xOff;
	bF.data[1] = (bF.data[1] * yDis) + yOff;
	bF.data[2] = (bF.data[2] * zDis) + zOff;
	b->data[0] = (int16_t) (bF.data[0] / magGain.data[0]);
	b->data[1] = (int16_t) (bF.data[1] / magGain.data[1]);
	b->data[2] = (int16_t) (bF.data[2] / magGain.data[2]);

	switch (direction) {
	case 0:
		r = r + 180.0f * t;
		if (r >= 360) {
			direction = 1;			
		}
	break;

	case 1:
		r = r - 180.0f * t;
		if (r <= 0) {
			direction = 2;
			w = 0;
		}
	break;		

	case 2:
		w = w + t;
		if (w > 3.0) {
			direction = 0;
			if (mode < 2) {
				mode++;
			} else {
				mode = 0;
			}
		}
	break;		
	}
}