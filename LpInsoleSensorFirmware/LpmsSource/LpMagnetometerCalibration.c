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

#include "LpMagnetometerCalibration.h"

#define LOW_PASS_ALPHA 0.01

float lpPsi, lpThe, lpPhi, lpInc;
int lpLoop = 0;
LpVector3f hIV;
LpMatrix3x3f sIM;

void initializeMCal(void)
{
	vectZero3x1(&hIV);
	createIdentity3x3(&sIM);
}

LpVector3f correctB(LpVector3f b)
{
	int i;
	LpVector3f x, y;
	
	for (i=0; i<3; ++i) x.data[i] = b.data[i] - hIV.data[i];
	
	for (i=0; i<3; ++i) {
		y.data[i] = sIM.data[i][0] * x.data[0] + sIM.data[i][1] * x.data[1] + sIM.data[i][2] * x.data[2];
	}
	
	return y;
}
	
float getFieldDirection(void)
{
	return lpPsi;
}

LpMatrix3x3f getSoftIronMatrix(void)
{
	return sIM;
}

LpVector3f getHardIronOffset(void)
{
	return hIV;
}

void setSoftIronMatrix(LpMatrix3x3f m)
{
	sIM = m;
}

void setHardIronOffset(LpVector3f v)
{
	hIV = v;
}

void getReferenceYZ(LpVector3f b, LpVector3f a, LpVector3f *r, float *inc)
{
	float sinR, cosR;
	float sZ, phi, the;
	float magic0, magic1, magic2, magic3, magic4;
	const float r2d = 57.2958f;	
	const float d2r = 0.01745f;

	if (a.data[2] >= 0.0f) sZ = 1.0f; else sZ = -1.0f;

	phi = (float) atan2(a.data[1], sZ * sqrt(a.data[2] * a.data[2])) * r2d;
	
	sinR = (float) sin(phi * d2r);
	cosR = (float) cos(phi * d2r);

	magic0 = b.data[1] * cosR - b.data[2] * sinR;
	magic1 = b.data[1] * sinR + b.data[2] * cosR;
	magic2 = a.data[1] * sinR + a.data[2] * cosR;
	
	if (magic2 == 0.0F) magic2 = 1e-10f;
	the = (float) atan(-a.data[0] / magic2) * r2d;
		
	sinR = (float) sin(the * d2r);
	cosR = (float) cos(the * d2r);	
		
	magic3 = b.data[0] * cosR + magic1 * sinR;
	magic4 = -b.data[0] * sinR + magic1 * cosR;	
	
	*inc = (float) atan2(magic4, sqrt(magic3 * magic3 + magic0 * magic0)) * r2d;	

	r->data[0] = phi;
	r->data[1] = sqrt(magic3 * magic3 + magic0 * magic0);
	r->data[2] = -magic4;
}

void bCalOrientationFromAccMag(LpVector3f b, LpVector3f a, LpVector3f *r, float *inc)
{
	float sinR, cosR;
	float sZ, phi, psi, the;
	float magic0, magic1, magic2, magic3, magic4;
	const float r2d = 57.2958f;	
	const float d2r = 0.01745f;

	if (a.data[2] >= 0.0f) sZ = 1.0f; else sZ = -1.0f;

	phi = (float) atan2(a.data[1], sZ * sqrt(a.data[2] * a.data[2])) * r2d;
	
	sinR = (float) sin(phi * d2r);
	cosR = (float) cos(phi * d2r);

	magic0 = b.data[1] * cosR - b.data[2] * sinR;
	magic1 = b.data[1] * sinR + b.data[2] * cosR;
	magic2 = a.data[1] * sinR + a.data[2] * cosR;
	
	if (magic2 == 0.0F) magic2 = 1e-10f;
	the = (float) atan(-a.data[0] / magic2) * r2d;
		
	sinR = (float) sin(the * d2r);
	cosR = (float) cos(the * d2r);	
		
	magic3 = b.data[0] * cosR + magic1 * sinR;
	magic4 = -b.data[0] * sinR + magic1 * cosR;	

	psi = (float) atan2(-magic0, magic3) * r2d; 
	
	*inc = (float) atan2(magic4, sqrt(magic3 * magic3 + magic0 * magic0)) * r2d;
	
	if (lpLoop == 0) {
		lpPhi = phi;
		lpThe = the;
		lpPsi = psi;
		lpInc = *inc;

		lpLoop = 1;
	}

	modLpFilter(phi, &lpPhi);
	modLpFilter(the, &lpThe);

	if (lpThe > 90.0f) {
		lpThe = 180.0f - lpThe;
	}
	
	if (lpThe < -90.0f) {
		lpThe = -180.0f - lpThe;
	}

	modLpFilter(psi, &lpPsi);
	modLpFilter(*inc, &lpInc);

	*inc = lpInc;

	r->data[0] = lpPhi;
	r->data[1] = lpThe;
	r->data[2] = lpPsi;
}

void modLpFilter(float a, float *lpA)
{
	float tA;

	tA = a - *lpA;

	if (tA > 180.0f) tA -= 360.0f;
	if (tA < -180.0f) tA += 360.0f;

	*lpA += LOW_PASS_ALPHA * tA;

	if (*lpA > 180.0f) *lpA -= 360.0f;
	if (*lpA < -180.0f) *lpA += 360.0f;
}