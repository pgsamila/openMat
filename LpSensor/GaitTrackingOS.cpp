#include "GaitTracking.h"

void resetPeakDetector(PeakDetector *pd)
{
}

void initIntegrator(LinAccIntegrator *lai)
{
}

void resetIntegrator(LinAccIntegrator *lai)
{
}

float lpFilter(float v, float *c, float a)
{
	return 0.0f;
}

float hpFilter(float v, float *c, float a)
{
	return 0.0f;
}

void integrateAcceleration(float T, float acc, LinAccIntegrator *lai, int rP)
{
}

void fitab(float *x, float *y, int n, float *a, float *b)
{
}

void initRB(RingBuffer *rb, int l)
{
}

void printRB(RingBuffer rb) 
{
}

void addRB(RingBuffer *rb, float v)
{
}

float getRBAvg(RingBuffer rb)
{
	return 0.0f;
}

void getRBAvgD(PeakDetector *pd, RingBuffer rb, float *b0, float *b1, float t, float acc, float h)
{
}

GaitTracking::GaitTracking(void)
{
}

void GaitTracking::update(ImuData *d)
{					
}