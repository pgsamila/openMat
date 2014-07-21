/***********************************************************************
** Module for heave motion detection
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef HEAVE_MOTION
#define HEAVE_MOTION

#include "LpMatrix.h"

// Ring buffer for data acquisition
typedef struct _RingBuffer {
	float v[128];
	int l;
	int i;
} RingBuffer;


// Ring buffer initialization
void initRB(RingBuffer *rb, int l);

// Adds new element to ring buffer
void addRB(RingBuffer *rb, float v);

// Retrieves ring buffer average
float getRBAvg(RingBuffer rb);

// Applies low-pass filter
float lpFilter(float v, float *c, float a);

// Applies high-pass filter
float hpFilter(float v, float *c, float a);

// Calculates heave motion
void calculateHeaveMotion(float T);

// Initializes heave motion calculation
void initHeaveMotion(void);

#endif