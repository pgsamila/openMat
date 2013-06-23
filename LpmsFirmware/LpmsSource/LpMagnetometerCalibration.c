/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpMagnetometerCalibration.h"

#define LOW_PASS_ALPHA 0.01

float fPsi, fThe, fPhi;
float fLPPsi, fLPThe, fLPPhi;
float fdelta;
float fLPdelta;
float fBpx, fBpy, fBpz;
float fBcx, fBcy, fBcz;
float fBfx, fBfy, fBfz;
float fGpx, fGpy, fGpz;
float fVx, fVy, fVz;
int loopcounter;
float xfinvW[3][3], *finvW[3];
float xA[3][3], *A[3];
float xinvA[3][3], *invA[3];

void initializeMCal(void)
{
	int i;

	for (i = 0; i < 3; i++) {
		finvW[i] = xfinvW[i];
		A[i] = xA[i];
		invA[i] = xinvA[i];
	}

	loopcounter = 0;
}

void correctB(float *bX, float *bY, float *bZ)
{
	fBpx = *bX;
	fBpy = *bY;
	fBpz = *bZ;
	
	fInvertHardandSoftIron();

	*bX = fBcx;
	*bY = fBcy;
	*bZ = fBcz;
}
	
float getFieldDirection(void)
{
	return fLPPsi;
}

void getSoftIronMatrix(LpMatrix3x3f *m)
{
	int i, j;
	
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			m->data[i][j] = finvW[i][j];
		}
	}
}

void getHardIronOffset(LpVector3f *o)
{
	o->data[0] = fVx;
	o->data[1] = fVy;
	o->data[2] = fVz;
}

void setSoftIronMatrix(LpMatrix3x3f m)
{
	int i, j;
	
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			finvW[i][j] = m.data[i][j];
		}
	}
}

void setHardIronOffset(LpVector3f o)
{
	fVx = o.data[0];
	fVy = o.data[1];
	fVz = o.data[2];
}

void fInvertHardandSoftIron()
{
	float ftmpx, ftmpy, ftmpz;

	fBcx = fBpx - fVx;
	fBcy = fBpy - fVy;
	fBcz = fBpz - fVz;

	ftmpx = finvW[0][0] * fBcx + finvW[0][1] * fBcy + finvW[0][2] * fBcz;
	ftmpy = finvW[1][0] * fBcx + finvW[1][1] * fBcy + finvW[1][2] * fBcz;
	ftmpz = finvW[2][0] * fBcx + finvW[2][1] * fBcy + finvW[2][2] * fBcz;

	fBcx = ftmpx; 
	fBcy = ftmpy;	
	fBcz = ftmpz;	
}

void getReferenceYZ(float fBx, float fBy, float fBz, 
	float fGx, float fGy, float fGz,
	float *refY, float *refZ, float *bInc)
{
	float sinAngle, cosAngle;

	fPhi = atan2(fGy, ((fGz >= 0.0F) ? 1.0F : -1.0F) * sqrt(fGz * fGz)) * RadToDeg;
	
	sinAngle = sin(fPhi * DegToRad);
	cosAngle = cos(fPhi * DegToRad);
	
	fBfy = fBy * cosAngle - fBz * sinAngle;
	fBz = fBy * sinAngle + fBz * cosAngle;
	fGz = fGy * sinAngle + fGz * cosAngle;

	if (fGz == 0.0f) fGz = 1e-10f;
	fThe = atan(-fGx / fGz) * RadToDeg;
	
	sinAngle = sin(fThe * DegToRad);
	cosAngle = cos(fThe * DegToRad);
	
	fBfx = fBx * cosAngle + fBz * sinAngle;
	fBfz = -fBx * sinAngle + fBz * cosAngle;

	*refY = sqrt(fBfx * fBfx + fBfy * fBfy);
	*refZ = -fBfz;

	*bInc = atan2(fBfz, sqrt(fBfx * fBfx + fBfy * fBfy)) * RadToDeg;
}

void feCompass(float fBx, float fBy, float fBz, 
	float fGx, float fGy, float fGz, 
	float *bInc,
	float *phiOut, float *thetaOut, float *psiOut)
{
	float sinAngle, cosAngle;

	fPhi = atan2(fGy, ((fGz >= 0.0f) ? 1.0f : -1.0f) * sqrt(fGz * fGz)) * RadToDeg;

	sinAngle = sin(fPhi * DegToRad);
	cosAngle = cos(fPhi * DegToRad);

	fBfy = fBy * cosAngle - fBz * sinAngle;
	fBz = fBy * sinAngle + fBz * cosAngle;
	fGz = fGy * sinAngle + fGz * cosAngle;

	if (fGz == 0.0f) fGz = 1e-10f;
	fThe = atan(-fGx / fGz) * RadToDeg;

	sinAngle = sin(fThe * DegToRad);
	cosAngle = cos(fThe * DegToRad);

	fBfx = fBx * cosAngle + fBz * sinAngle;
	fBfz = -fBx * sinAngle + fBz * cosAngle;

	fPsi = atan2(-fBfy, fBfx) * RadToDeg; 

	fdelta = atan2(fBfz, sqrt(fBfx * fBfx + fBfy * fBfy)) * RadToDeg;

	if (loopcounter == 0) {
		fLPPhi = fPhi;
		fLPThe = fThe;
		fLPPsi = fPsi;
		fLPdelta = fdelta;

		loopcounter = 1;
	}

	fModuloLPF(fPhi, &fLPPhi);
	fModuloLPF(fThe, &fLPThe);

	if (fLPThe > 90.0F) {
		fLPThe = 180.0F - fLPThe;
	}
	
	if (fLPThe < -90.0F) {
		fLPThe = -180.0F - fLPThe;
	}

	fModuloLPF(fPsi, &fLPPsi);
	fModuloLPF(fdelta, &fLPdelta);

	*bInc = fdelta;
	*phiOut = fLPPhi * DegToRad;
	*thetaOut = fLPThe * DegToRad;
	*psiOut = fLPPsi * DegToRad;
}

void fModuloLPF(float fAngle, float *pfLPFAngle)
{
	float ftmpAngle;

	ftmpAngle = fAngle - *pfLPFAngle;

	if (ftmpAngle > 180.0f) ftmpAngle -= 360.0f;
	if (ftmpAngle < -180.0f) ftmpAngle += 360.0f;

	*pfLPFAngle += LOW_PASS_ALPHA * ftmpAngle;

	if (*pfLPFAngle > 180.0F) *pfLPFAngle -= 360.0F;
	if (*pfLPFAngle < -180.0F) *pfLPFAngle += 360.0F;
}