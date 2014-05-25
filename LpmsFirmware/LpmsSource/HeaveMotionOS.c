#include "LpmsFactorySetting.h"
#ifdef USE_HEAVEMOTION

#include "HeaveMotion.h"

extern LpVector3f heaveOutput;
extern LpVector4f q;
extern LpVector3f a;

float lpFilter(float v, float *c, float a)
{
	return 0.0f;
}

float hpFilter(float v, float *c, float a)
{
	return 0.0f;
}

void initHeaveMotion(void)
{
}

/*	INPUT
	a: Accelerometer data
	q: Orientation quaternion

	OUTPUT
	heaveOutput.data[2]: Z-axis translation */
void calculateHeaveMotion(float T)
{
}

#endif