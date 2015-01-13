#ifndef RING_BUFFER
#define RING_BUFFER

#include "math.h"

#define RB_MAX_SAMPLES 1024

typedef struct _RingBuffer {
	float v[1024];
	int l;
	int i;
} RingBuffer;

#ifdef __cplusplus
extern "C" {
#endif

void initRB(RingBuffer *rb, int l);
void printRB(RingBuffer rb);
void addRB(RingBuffer *rb, float v);
float getRBAvg(RingBuffer rb);
void getRBSpectrum(float bFreq, float eFreq, float fStep, float sFreq, RingBuffer *rb, float *fSpec, int *sSize);
void hamming(float *in, int inSize, float *out);
void calcFft(float *in, int inSize, float aFreq, float sFreq, float *amp);

#ifdef __cplusplus
}
#endif 

#endif