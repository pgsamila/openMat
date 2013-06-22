#include "LpFilterCVersion.h"
             
void initFilterState(LpFilterState* fs,
	float aCGain, float aC,
	float bCGain, float bC,
	float aRX, float aRY, float aRZ,
	float bRX, float bRY, float bRZ)
{
}

void QuaternionEKFInit(float aCGain, float aC,
	float bCGain, float bC,
	float aRX, float aRY, float aRZ,
	float bRX, float bRY, float bRZ)
{		
}
	
void QuaternionEKF(float aX, float aY, float aZ,
	float gX, float gY, float gZ,
	float bX, float bY, float bZ,
	float oBX, float oBY, float oBZ,	
	float T,
	int useAC, int useBC,
	float *oQ0, float *oQ1, float *oQ2, float *oQ3,
	int useGyrT, 
	float gyrTx, float gyrTy, float gyrTz,
	int *magOutOfRange,
	float fitError)
{
}

void updateFilter(LpFilterState *fs,
	float aX, float aY, float aZ,
	float gX, float gY, float gZ,
	float bX, float bY, float bZ,
	float T,
	int useAC, int useBC,
	float *oQ0, float *oQ1, float *oQ2, float *oQ3,
	int useGyrT, 
	float gyrTx, float gyrTy, float gyrTz)
{
}