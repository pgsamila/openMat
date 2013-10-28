#include "RingBuffer.h"

#include <stdio.h>

#define PI 3.1415f

void initRB(RingBuffer *rb, int l)
{
	int i;

	rb->l = l;
	rb->i = 0;
	
	for (i=0; i<rb->l; ++i) rb->v[i] = 0.0f;
}

void printRB(RingBuffer rb) 
{
	int i;

	for (i=0; i<rb.l; ++i) {
		printf("%f, ", rb.v[i]);
	}
	printf("\n");
}

void addRB(RingBuffer *rb, float v)
{
	if (rb->i > 0) if (rb->v[rb->i - 1] == v) return;
	
	rb->v[rb->i] = v;
	if (rb->i<rb->l) ++rb->i; else rb->i = 0;
}

float getRBAvg(RingBuffer rb)
{
	float mi = 99.9f;
	float ma = -99.9f;
	float s = 0.0f;
	int i;

	for (i=0; i<rb.l; ++i) {
		if (rb.v[i] > ma) ma = rb.v[i];
		if (rb.v[i] < mi) mi = rb.v[i];
		s += rb.v[i];
	}
	
	return (s / (float) rb.l);
}

void getRBSpectrum(float bFreq, float eFreq, float fStep, float sFreq, RingBuffer *rb, float *fSpec, int *sSize)
{
	int i;
	float f;
	
	i = 0;
	f = bFreq;
	
	while (f < eFreq) {
		f += fStep;
	
		calcFft(rb->v, rb->l, f, sFreq, &fSpec[i]);
		++i;
	}
}

void hamming(float *in, int inSize, float *out)
{	
	int i;

	for (i=0; i<inSize; i++) {
		out[i] = in[i] * 0.5f * (1.0f - (float) cos(2.0f * PI * (float) i / (float)(inSize - 1)));
	}
}

void calcFft(float *in, int inSize, float aFreq, float sFreq, float *amp)
{
	float a, b, r, d;
	float buf[RB_MAX_SAMPLES];
	int i, tics;
	
	hamming(in, inSize, buf);
	
	a = 0.0;
	b = 0.0;

	d = aFreq / sFreq;
	tics = inSize;

	for (i = 0; i < tics; ++i) {
		r = 2.0f * PI * d * (float) i / (float) tics; 
		a = a + (float) buf[i] * (float) cos(r);
		b = b + (float) buf[i] * (float)sin(r);
	}

	a = a * 2.0f / (float) tics;
	b = b * 2.0f / (float) tics;

	*amp = (float) fabs(sqrt(a*a + b*b));
}