/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "LpMatrix.h"

#define FLT_EPSILON 1.192092896e-07f
#define SQRT_MAGIC_F 0x5f3759df

const float r2d = 57.2958f;

float fastSqrt(const float x)
{
	const float xhalf = 0.5f*x;
	
	union {
		float x;
		int i;
	} u;

	u.x = x;
	u.i = SQRT_MAGIC_F - (u.i >> 1);
	
	return x*u.x*(1.5f - xhalf*u.x*u.x);
}

float invSqrt(float x)
{
	float xhalf = 0.5f*x;
	int i = *(int*)&x;

	i = 0x5f3759df - (i>>1);
	x = *(float*)&i;
	x = x * (1.5f-xhalf*x*x);

	return x;
}

float fastCos(float x)
{
	float cos;

	x += 1.57079632;

	if (x > 3.14159265) x -= 6.28318531;
	
	if (x < 0) {
		cos = 1.27323954 * x + 0.405284735 * x * x;
		
		
		if (cos < 0) {
			cos = .225 * (cos *-cos - cos) + cos;
		} else {
			cos = .225 * (cos * cos - cos) + cos;
		}
	} else {
		cos = 1.27323954 * x - 0.405284735 * x * x;
		
		if (cos < 0) {
			cos = .225 * (cos *-cos - cos) + cos;
		} else {
			cos = .225 * (cos * cos - cos) + cos;
		}
	}

	return cos;
}

float fastSin(float x)
{
	float sin;

	if (x < 0) {
		sin = 1.27323954 * x + .405284735 * x * x;
	
		if (sin < 0) {
			sin = .225 * (sin *-sin - sin) + sin;
		} else {
			sin = .225 * (sin * sin - sin) + sin;
		}
	} else {
		sin = 1.27323954 * x - 0.405284735 * x * x;
		
		if (sin < 0) {
			sin = .225 * (sin *-sin - sin) + sin;
		} else {
			sin = .225 * (sin * sin - sin) + sin;
		}
	}

	return sin;
}

int matAdd3x3(LpMatrix3x3f* src1, LpMatrix3x3f* src2, LpMatrix3x3f* dest)
{
	int i, j;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++ ) {
			dest->data[i][j] = src1->data[i][j] + src2->data[i][j];
		}
	}

	return 1;
}

int matAdd4x4(LpMatrix4x4f* src1, LpMatrix4x4f* src2, LpMatrix4x4f* dest)
{
	int i, j;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++ ) {
			dest->data[i][j] = src1->data[i][j] + src2->data[i][j];
		}
	}

	return 1;
}

int matMult3x3(LpMatrix3x3f* src1, LpMatrix3x3f* src2, LpMatrix3x3f* dest)
{
	int i, j;
	LpMatrix3x3f result;

	result.data[0][0] = src1->data[0][0]*src2->data[0][0] + src1->data[0][1]*src2->data[1][0] + src1->data[0][2]*src2->data[2][0];
	result.data[0][1] = src1->data[0][0]*src2->data[0][1] + src1->data[0][1]*src2->data[1][1] + src1->data[0][2]*src2->data[2][1];
	result.data[0][2] = src1->data[0][0]*src2->data[0][2] + src1->data[0][1]*src2->data[1][2] + src1->data[0][2]*src2->data[2][2];

	result.data[1][0] = src1->data[1][0]*src2->data[0][0] + src1->data[1][1]*src2->data[1][0] + src1->data[1][2]*src2->data[2][0];
	result.data[1][1] = src1->data[1][0]*src2->data[0][1] + src1->data[1][1]*src2->data[1][1] + src1->data[1][2]*src2->data[2][1];
	result.data[1][2] = src1->data[1][0]*src2->data[0][2] + src1->data[1][1]*src2->data[1][2] + src1->data[1][2]*src2->data[2][2];

	result.data[2][0] = src1->data[2][0]*src2->data[0][0] + src1->data[2][1]*src2->data[1][0] + src1->data[2][2]*src2->data[2][0];
	result.data[2][1] = src1->data[2][0]*src2->data[0][1] + src1->data[2][1]*src2->data[1][1] + src1->data[2][2]*src2->data[2][1];
	result.data[2][2] = src1->data[2][0]*src2->data[0][2] + src1->data[2][1]*src2->data[1][2] + src1->data[2][2]*src2->data[2][2];

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++ ) {
			dest->data[i][j] = result.data[i][j];
		}
	}

	return 1;
}

int matMult3x4to4x4(LpMatrix3x4f* src1, LpMatrix4x4f* src2, LpMatrix3x4f* dest)
{
	int i, j;
	LpMatrix3x4f result;

	result.data[0][0] = src1->data[0][0]*src2->data[0][0] + src1->data[0][1]*src2->data[1][0] + src1->data[0][2]*src2->data[2][0] + src1->data[0][3]*src2->data[3][0];
	result.data[0][1] = src1->data[0][0]*src2->data[0][1] + src1->data[0][1]*src2->data[1][1] + src1->data[0][2]*src2->data[2][1] + src1->data[0][3]*src2->data[3][1];
	result.data[0][2] = src1->data[0][0]*src2->data[0][2] + src1->data[0][1]*src2->data[1][2] + src1->data[0][2]*src2->data[2][2] + src1->data[0][3]*src2->data[3][2];
	result.data[0][3] = src1->data[0][0]*src2->data[0][3] + src1->data[0][1]*src2->data[1][3] + src1->data[0][2]*src2->data[2][3] + src1->data[0][3]*src2->data[3][3];

	result.data[1][0] = src1->data[1][0]*src2->data[0][0] + src1->data[1][1]*src2->data[1][0] + src1->data[1][2]*src2->data[2][0] + src1->data[1][3]*src2->data[3][0];
	result.data[1][1] = src1->data[1][0]*src2->data[0][1] + src1->data[1][1]*src2->data[1][1] + src1->data[1][2]*src2->data[2][1] + src1->data[1][3]*src2->data[3][1];
	result.data[1][2] = src1->data[1][0]*src2->data[0][2] + src1->data[1][1]*src2->data[1][2] + src1->data[1][2]*src2->data[2][2] + src1->data[1][3]*src2->data[3][2];
	result.data[1][3] = src1->data[1][0]*src2->data[0][3] + src1->data[1][1]*src2->data[1][3] + src1->data[1][2]*src2->data[2][3] + src1->data[1][3]*src2->data[3][3];

	result.data[2][0] = src1->data[2][0]*src2->data[0][0] + src1->data[2][1]*src2->data[1][0] + src1->data[2][2]*src2->data[2][0] + src1->data[2][3]*src2->data[3][0];
	result.data[2][1] = src1->data[2][0]*src2->data[0][1] + src1->data[2][1]*src2->data[1][1] + src1->data[2][2]*src2->data[2][1] + src1->data[2][3]*src2->data[3][1];
	result.data[2][2] = src1->data[2][0]*src2->data[0][2] + src1->data[2][1]*src2->data[1][2] + src1->data[2][2]*src2->data[2][2] + src1->data[2][3]*src2->data[3][2];
	result.data[2][3] = src1->data[2][0]*src2->data[0][3] + src1->data[2][1]*src2->data[1][3] + src1->data[2][2]*src2->data[2][3] + src1->data[2][3]*src2->data[3][3];

	for (i = 0; i < 3; i++)
	{
	  for (j = 0; j < 4; j++ )
	  {
			dest->data[i][j] = result.data[i][j];
	  }
	}

	return 1;
}

int matMult3x4to4x3( LpMatrix3x4f* src1, LpMatrix4x3f* src2, LpMatrix3x3f* dest )
{
	int i, j;
	LpMatrix3x3f result;

	result.data[0][0] = src1->data[0][0]*src2->data[0][0] + src1->data[0][1]*src2->data[1][0] + src1->data[0][2]*src2->data[2][0] + src1->data[0][3]*src2->data[3][0];
	result.data[0][1] = src1->data[0][0]*src2->data[0][1] + src1->data[0][1]*src2->data[1][1] + src1->data[0][2]*src2->data[2][1] + src1->data[0][3]*src2->data[3][1];
	result.data[0][2] = src1->data[0][0]*src2->data[0][2] + src1->data[0][1]*src2->data[1][2] + src1->data[0][2]*src2->data[2][2] + src1->data[0][3]*src2->data[3][2];

	result.data[1][0] = src1->data[1][0]*src2->data[0][0] + src1->data[1][1]*src2->data[1][0] + src1->data[1][2]*src2->data[2][0] + src1->data[1][3]*src2->data[3][0];
	result.data[1][1] = src1->data[1][0]*src2->data[0][1] + src1->data[1][1]*src2->data[1][1] + src1->data[1][2]*src2->data[2][1] + src1->data[1][3]*src2->data[3][1];
	result.data[1][2] = src1->data[1][0]*src2->data[0][2] + src1->data[1][1]*src2->data[1][2] + src1->data[1][2]*src2->data[2][2] + src1->data[1][3]*src2->data[3][2];

	result.data[2][0] = src1->data[2][0]*src2->data[0][0] + src1->data[2][1]*src2->data[1][0] + src1->data[2][2]*src2->data[2][0] + src1->data[2][3]*src2->data[3][0];
	result.data[2][1] = src1->data[2][0]*src2->data[0][1] + src1->data[2][1]*src2->data[1][1] + src1->data[2][2]*src2->data[2][1] + src1->data[2][3]*src2->data[3][1];
	result.data[2][2] = src1->data[2][0]*src2->data[0][2] + src1->data[2][1]*src2->data[1][2] + src1->data[2][2]*src2->data[2][2] + src1->data[2][3]*src2->data[3][2];


	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++ ) {
			dest->data[i][j] = result.data[i][j];
		}
	}

	return 1;
}

int matMult4x4(LpMatrix4x4f* src1, LpMatrix4x4f* src2, LpMatrix4x4f* dest)
{
	int i, j;
	LpMatrix4x4f result;

	result.data[0][0] = src1->data[0][0]*src2->data[0][0] + src1->data[0][1]*src2->data[1][0] + src1->data[0][2]*src2->data[2][0] + src1->data[0][3]*src2->data[3][0];
	result.data[0][1] = src1->data[0][0]*src2->data[0][1] + src1->data[0][1]*src2->data[1][1] + src1->data[0][2]*src2->data[2][1] + src1->data[0][3]*src2->data[3][1];
	result.data[0][2] = src1->data[0][0]*src2->data[0][2] + src1->data[0][1]*src2->data[1][2] + src1->data[0][2]*src2->data[2][2] + src1->data[0][3]*src2->data[3][2];
	result.data[0][3] = src1->data[0][0]*src2->data[0][3] + src1->data[0][1]*src2->data[1][3] + src1->data[0][2]*src2->data[2][3] + src1->data[0][3]*src2->data[3][3];

	result.data[1][0] = src1->data[1][0]*src2->data[0][0] + src1->data[1][1]*src2->data[1][0] + src1->data[1][2]*src2->data[2][0] + src1->data[1][3]*src2->data[3][0];
	result.data[1][1] = src1->data[1][0]*src2->data[0][1] + src1->data[1][1]*src2->data[1][1] + src1->data[1][2]*src2->data[2][1] + src1->data[1][3]*src2->data[3][1];
	result.data[1][2] = src1->data[1][0]*src2->data[0][2] + src1->data[1][1]*src2->data[1][2] + src1->data[1][2]*src2->data[2][2] + src1->data[1][3]*src2->data[3][2];
	result.data[1][3] = src1->data[1][0]*src2->data[0][3] + src1->data[1][1]*src2->data[1][3] + src1->data[1][2]*src2->data[2][3] + src1->data[1][3]*src2->data[3][3];

	result.data[2][0] = src1->data[2][0]*src2->data[0][0] + src1->data[2][1]*src2->data[1][0] + src1->data[2][2]*src2->data[2][0] + src1->data[2][3]*src2->data[3][0];
	result.data[2][1] = src1->data[2][0]*src2->data[0][1] + src1->data[2][1]*src2->data[1][1] + src1->data[2][2]*src2->data[2][1] + src1->data[2][3]*src2->data[3][1];
	result.data[2][2] = src1->data[2][0]*src2->data[0][2] + src1->data[2][1]*src2->data[1][2] + src1->data[2][2]*src2->data[2][2] + src1->data[2][3]*src2->data[3][2];
	result.data[2][3] = src1->data[2][0]*src2->data[0][3] + src1->data[2][1]*src2->data[1][3] + src1->data[2][2]*src2->data[2][3] + src1->data[2][3]*src2->data[3][3];

	result.data[3][0] = src1->data[3][0]*src2->data[0][0] + src1->data[3][1]*src2->data[1][0] + src1->data[3][2]*src2->data[2][0] + src1->data[3][3]*src2->data[3][0];
	result.data[3][1] = src1->data[3][0]*src2->data[0][1] + src1->data[3][1]*src2->data[1][1] + src1->data[3][2]*src2->data[2][1] + src1->data[3][3]*src2->data[3][1];
	result.data[3][2] = src1->data[3][0]*src2->data[0][2] + src1->data[3][1]*src2->data[1][2] + src1->data[3][2]*src2->data[2][2] + src1->data[3][3]*src2->data[3][2];
	result.data[3][3] = src1->data[3][0]*src2->data[0][3] + src1->data[3][1]*src2->data[1][3] + src1->data[3][2]*src2->data[2][3] + src1->data[3][3]*src2->data[3][3];

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			dest->data[i][j] = result.data[i][j];
		}
	}

	return 1;
}

int matMult4x4to4x3( LpMatrix4x4f* src1, LpMatrix4x3f* src2, LpMatrix4x3f* dest )
{
	int i, j;
	LpMatrix4x3f result;

	result.data[0][0] = src1->data[0][0]*src2->data[0][0] + src1->data[0][1]*src2->data[1][0] + src1->data[0][2]*src2->data[2][0] + src1->data[0][3]*src2->data[3][0];
	result.data[0][1] = src1->data[0][0]*src2->data[0][1] + src1->data[0][1]*src2->data[1][1] + src1->data[0][2]*src2->data[2][1] + src1->data[0][3]*src2->data[3][1];
	result.data[0][2] = src1->data[0][0]*src2->data[0][2] + src1->data[0][1]*src2->data[1][2] + src1->data[0][2]*src2->data[2][2] + src1->data[0][3]*src2->data[3][2];

	result.data[1][0] = src1->data[1][0]*src2->data[0][0] + src1->data[1][1]*src2->data[1][0] + src1->data[1][2]*src2->data[2][0] + src1->data[1][3]*src2->data[3][0];
	result.data[1][1] = src1->data[1][0]*src2->data[0][1] + src1->data[1][1]*src2->data[1][1] + src1->data[1][2]*src2->data[2][1] + src1->data[1][3]*src2->data[3][1];
	result.data[1][2] = src1->data[1][0]*src2->data[0][2] + src1->data[1][1]*src2->data[1][2] + src1->data[1][2]*src2->data[2][2] + src1->data[1][3]*src2->data[3][2];

	result.data[2][0] = src1->data[2][0]*src2->data[0][0] + src1->data[2][1]*src2->data[1][0] + src1->data[2][2]*src2->data[2][0] + src1->data[2][3]*src2->data[3][0];
	result.data[2][1] = src1->data[2][0]*src2->data[0][1] + src1->data[2][1]*src2->data[1][1] + src1->data[2][2]*src2->data[2][1] + src1->data[2][3]*src2->data[3][1];
	result.data[2][2] = src1->data[2][0]*src2->data[0][2] + src1->data[2][1]*src2->data[1][2] + src1->data[2][2]*src2->data[2][2] + src1->data[2][3]*src2->data[3][2];

	result.data[3][0] = src1->data[3][0]*src2->data[0][0] + src1->data[3][1]*src2->data[1][0] + src1->data[3][2]*src2->data[2][0] + src1->data[3][3]*src2->data[3][0];
	result.data[3][1] = src1->data[3][0]*src2->data[0][1] + src1->data[3][1]*src2->data[1][1] + src1->data[3][2]*src2->data[2][1] + src1->data[3][3]*src2->data[3][1];
	result.data[3][2] = src1->data[3][0]*src2->data[0][2] + src1->data[3][1]*src2->data[1][2] + src1->data[3][2]*src2->data[2][2] + src1->data[3][3]*src2->data[3][2];

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 3; j++ ) {
			dest->data[i][j] = result.data[i][j];
		}
	}

	return 1;
}

int matMult4x3to3x3(LpMatrix4x3f* src1, LpMatrix3x3f* src2, LpMatrix4x3f* dest)
{
	int i, j;
	LpMatrix4x3f result;

	result.data[0][0] = src1->data[0][0]*src2->data[0][0] + src1->data[0][1]*src2->data[1][0] + src1->data[0][2]*src2->data[2][0] ;
	result.data[0][1] = src1->data[0][0]*src2->data[0][1] + src1->data[0][1]*src2->data[1][1] + src1->data[0][2]*src2->data[2][1] ;
	result.data[0][2] = src1->data[0][0]*src2->data[0][2] + src1->data[0][1]*src2->data[1][2] + src1->data[0][2]*src2->data[2][2] ;

	result.data[1][0] = src1->data[1][0]*src2->data[0][0] + src1->data[1][1]*src2->data[1][0] + src1->data[1][2]*src2->data[2][0] ;
	result.data[1][1] = src1->data[1][0]*src2->data[0][1] + src1->data[1][1]*src2->data[1][1] + src1->data[1][2]*src2->data[2][1] ;
	result.data[1][2] = src1->data[1][0]*src2->data[0][2] + src1->data[1][1]*src2->data[1][2] + src1->data[1][2]*src2->data[2][2] ;

	result.data[2][0] = src1->data[2][0]*src2->data[0][0] + src1->data[2][1]*src2->data[1][0] + src1->data[2][2]*src2->data[2][0] ;
	result.data[2][1] = src1->data[2][0]*src2->data[0][1] + src1->data[2][1]*src2->data[1][1] + src1->data[2][2]*src2->data[2][1] ;
	result.data[2][2] = src1->data[2][0]*src2->data[0][2] + src1->data[2][1]*src2->data[1][2] + src1->data[2][2]*src2->data[2][2] ;

	result.data[3][0] = src1->data[3][0]*src2->data[0][0] + src1->data[3][1]*src2->data[1][0] + src1->data[3][2]*src2->data[2][0] ;
	result.data[3][1] = src1->data[3][0]*src2->data[0][1] + src1->data[3][1]*src2->data[1][1] + src1->data[3][2]*src2->data[2][1] ;
	result.data[3][2] = src1->data[3][0]*src2->data[0][2] + src1->data[3][1]*src2->data[1][2] + src1->data[3][2]*src2->data[2][2] ;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 3; j++ ) {
			dest->data[i][j] = result.data[i][j];
		}
	}

	return 1;
}

int matMult4x3to3x4(LpMatrix4x3f* src1, LpMatrix3x4f* src2, LpMatrix4x4f* dest)
{
	int i, j;
	LpMatrix4x4f result;

	result.data[0][0] = src1->data[0][0]*src2->data[0][0] + src1->data[0][1]*src2->data[1][0] + src1->data[0][2]*src2->data[2][0];
	result.data[0][1] = src1->data[0][0]*src2->data[0][1] + src1->data[0][1]*src2->data[1][1] + src1->data[0][2]*src2->data[2][1];
	result.data[0][2] = src1->data[0][0]*src2->data[0][2] + src1->data[0][1]*src2->data[1][2] + src1->data[0][2]*src2->data[2][2];
	result.data[0][3] = src1->data[0][0]*src2->data[0][3] + src1->data[0][1]*src2->data[1][3] + src1->data[0][2]*src2->data[2][3];

	result.data[1][0] = src1->data[1][0]*src2->data[0][0] + src1->data[1][1]*src2->data[1][0] + src1->data[1][2]*src2->data[2][0];
	result.data[1][1] = src1->data[1][0]*src2->data[0][1] + src1->data[1][1]*src2->data[1][1] + src1->data[1][2]*src2->data[2][1];
	result.data[1][2] = src1->data[1][0]*src2->data[0][2] + src1->data[1][1]*src2->data[1][2] + src1->data[1][2]*src2->data[2][2];
	result.data[1][3] = src1->data[1][0]*src2->data[0][3] + src1->data[1][1]*src2->data[1][3] + src1->data[1][2]*src2->data[2][3];

	result.data[2][0] = src1->data[2][0]*src2->data[0][0] + src1->data[2][1]*src2->data[1][0] + src1->data[2][2]*src2->data[2][0] ;
	result.data[2][1] = src1->data[2][0]*src2->data[0][1] + src1->data[2][1]*src2->data[1][1] + src1->data[2][2]*src2->data[2][1] ;
	result.data[2][2] = src1->data[2][0]*src2->data[0][2] + src1->data[2][1]*src2->data[1][2] + src1->data[2][2]*src2->data[2][2] ;
	result.data[2][3] = src1->data[2][0]*src2->data[0][3] + src1->data[2][1]*src2->data[1][3] + src1->data[2][2]*src2->data[2][3] ;

	result.data[3][0] = src1->data[3][0]*src2->data[0][0] + src1->data[3][1]*src2->data[1][0] + src1->data[3][2]*src2->data[2][0] ;
	result.data[3][1] = src1->data[3][0]*src2->data[0][1] + src1->data[3][1]*src2->data[1][1] + src1->data[3][2]*src2->data[2][1] ;
	result.data[3][2] = src1->data[3][0]*src2->data[0][2] + src1->data[3][1]*src2->data[1][2] + src1->data[3][2]*src2->data[2][2] ;
	result.data[3][3] = src1->data[3][0]*src2->data[0][3] + src1->data[3][1]*src2->data[1][3] + src1->data[3][2]*src2->data[2][3] ;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++ ) {
			dest->data[i][j] = result.data[i][j];
		}
	}

	return 1;
}

float matInv3x3(LpMatrix3x3f* src, LpMatrix3x3f* dest)
{
	int i, j;
	float det;
	LpMatrix3x3f inverse;

	det = matDet3x3(src);

	inverse.data[0][0] = (src->data[2][2]*src->data[1][1]-src->data[2][1]*src->data[1][2])/det;
	inverse.data[0][1] = -(src->data[2][2]*src->data[0][1]-src->data[2][1]*src->data[0][2])/det;
	inverse.data[0][2] = (src->data[1][2]*src->data[0][1]-src->data[1][1]*src->data[0][2])/det;

	inverse.data[1][0] = -(src->data[2][2]*src->data[1][0]-src->data[2][0]*src->data[1][2])/det;
	inverse.data[1][1] = (src->data[2][2]*src->data[0][0]-src->data[2][0]*src->data[0][2])/det;
	inverse.data[1][2] = -(src->data[1][2]*src->data[0][0]-src->data[1][0]*src->data[0][2])/det;

	inverse.data[2][0] = (src->data[2][1]*src->data[1][0]-src->data[2][0]*src->data[1][1])/det;
	inverse.data[2][1] = -(src->data[2][1]*src->data[0][0]-src->data[2][0]*src->data[0][1])/det;
	inverse.data[2][2] = (src->data[1][1]*src->data[0][0]-src->data[1][0]*src->data[0][1])/det;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			dest->data[i][j] = inverse.data[i][j];
		}
	}

	return 1;
}

int matVectMult3(LpMatrix3x3f* matrix, LpVector3f* vector, LpVector3f* dest)
{
	int i;
	LpVector3f result;

	result.data[0] = matrix->data[0][0]*vector->data[0] + matrix->data[0][1]*vector->data[1] + matrix->data[0][2]*vector->data[2];
	result.data[1] = matrix->data[1][0]*vector->data[0] + matrix->data[1][1]*vector->data[1] + matrix->data[1][2]*vector->data[2];
	result.data[2] = matrix->data[2][0]*vector->data[0] + matrix->data[2][1]*vector->data[1] + matrix->data[2][2]*vector->data[2];

	for (i = 0; i < 3; i++) {
		dest->data[i] = result.data[i];
	}

	return 1;
}

int matVectMult4(LpMatrix4x4f* matrix, LpVector4f* vector, LpVector4f* dest)
{
	int i;
	LpVector4f result;

	result.data[0] = matrix->data[0][0]*vector->data[0] + matrix->data[0][1]*vector->data[1] + matrix->data[0][2]*vector->data[2] + matrix->data[0][3]*vector->data[3];
	result.data[1] = matrix->data[1][0]*vector->data[0] + matrix->data[1][1]*vector->data[1] + matrix->data[1][2]*vector->data[2] + matrix->data[1][3]*vector->data[3];
	result.data[2] = matrix->data[2][0]*vector->data[0] + matrix->data[2][1]*vector->data[1] + matrix->data[2][2]*vector->data[2] + matrix->data[2][3]*vector->data[3];
	result.data[3] = matrix->data[3][0]*vector->data[0] + matrix->data[3][1]*vector->data[1] + matrix->data[3][2]*vector->data[2] + matrix->data[3][3]*vector->data[3];
	
	for (i = 0; i < 4; i++) {
		dest->data[i] = result.data[i];
	}

	return 1;
}

int matVectMult3x4(LpMatrix3x4f* matrix, LpVector4f* vector, LpVector3f* dest)
{
	int i;
	LpVector4f result;

	result.data[0] = matrix->data[0][0]*vector->data[0] + matrix->data[0][1]*vector->data[1] + matrix->data[0][2]*vector->data[2] + matrix->data[0][3]*vector->data[3];
	result.data[1] = matrix->data[1][0]*vector->data[0] + matrix->data[1][1]*vector->data[1] + matrix->data[1][2]*vector->data[2] + matrix->data[1][3]*vector->data[3];
	result.data[2] = matrix->data[2][0]*vector->data[0] + matrix->data[2][1]*vector->data[1] + matrix->data[2][2]*vector->data[2] + matrix->data[2][3]*vector->data[3];
	
	for (i = 0; i < 3; i++) {
		dest->data[i] = result.data[i];
	}

	return 1;
}

int matVectMult4x3(LpMatrix4x3f* matrix, LpVector3f* vector, LpVector4f* dest)
{
	int i;
	LpVector4f result;

	result.data[0] = matrix->data[0][0]*vector->data[0] + matrix->data[0][1]*vector->data[1] + matrix->data[0][2]*vector->data[2];
	result.data[1] = matrix->data[1][0]*vector->data[0] + matrix->data[1][1]*vector->data[1] + matrix->data[1][2]*vector->data[2];
	result.data[2] = matrix->data[2][0]*vector->data[0] + matrix->data[2][1]*vector->data[1] + matrix->data[2][2]*vector->data[2];
	result.data[3] = matrix->data[3][0]*vector->data[0] + matrix->data[3][1]*vector->data[1] + matrix->data[3][2]*vector->data[2];
	
	for (i = 0; i < 4; i++) {
		dest->data[i] = result.data[i];
	}

	return 1;
}

float matDet3x3(LpMatrix3x3f* src)
{
	 	 
	return src->data[0][0]*(src->data[2][2]*src->data[1][1]-src->data[2][1]*src->data[1][2])
		- src->data[1][0]*(src->data[2][2]*src->data[0][1]-src->data[2][1]*src->data[0][2])
		+ src->data[2][0]*(src->data[1][2]*src->data[0][1]-src->data[1][1]*src->data[0][2]);
}

int matTrans3x3(LpMatrix3x3f* src, LpMatrix3x3f* dest)
{
	int i,j;
	LpMatrix3x3f temp;

	temp.data[0][0] = src->data[0][0];
	temp.data[1][0] = src->data[0][1];
	temp.data[2][0] = src->data[0][2];

	temp.data[0][1] = src->data[1][0];
	temp.data[1][1] = src->data[1][1];
	temp.data[2][1] = src->data[1][2];

	temp.data[0][2] = src->data[2][0];
	temp.data[1][2] = src->data[2][1];
	temp.data[2][2] = src->data[2][2];

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			dest->data[i][j] = temp.data[i][j];
		}
	}

	return 1;	 
}

int matTrans4x4(LpMatrix4x4f* src, LpMatrix4x4f* dest)
{
	int i,j;
	LpMatrix4x4f temp;

	temp.data[0][0] = src->data[0][0];
	temp.data[1][0] = src->data[0][1];
	temp.data[2][0] = src->data[0][2];
	temp.data[3][0] = src->data[0][3];

	temp.data[0][1] = src->data[1][0];
	temp.data[1][1] = src->data[1][1];
	temp.data[2][1] = src->data[1][2];
	temp.data[3][1] = src->data[1][3];

	temp.data[0][2] = src->data[2][0];
	temp.data[1][2] = src->data[2][1];
	temp.data[2][2] = src->data[2][2];
	temp.data[3][2] = src->data[2][3];

	temp.data[0][3] = src->data[3][0];
	temp.data[1][3] = src->data[3][1];
	temp.data[2][3] = src->data[3][2];
	temp.data[3][3] = src->data[3][3];

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			dest->data[i][j] = temp.data[i][j];
		}
	}

	return 1;	 
}

int matTrans3x4(LpMatrix3x4f* src, LpMatrix4x3f* dest)
{
	int i,j;
	LpMatrix4x3f temp;

	temp.data[0][0] = src->data[0][0];
	temp.data[1][0] = src->data[0][1];
	temp.data[2][0] = src->data[0][2];
	temp.data[3][0] = src->data[0][3];

	temp.data[0][1] = src->data[1][0];
	temp.data[1][1] = src->data[1][1];
	temp.data[2][1] = src->data[1][2];
	temp.data[3][1] = src->data[1][3];

	temp.data[0][2] = src->data[2][0];
	temp.data[1][2] = src->data[2][1];
	temp.data[2][2] = src->data[2][2];
	temp.data[3][2] = src->data[2][3];

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 3; j++) {
			dest->data[i][j] = temp.data[i][j];
		}
	}

	return 1;	 
}

int scalarMatMult3x3(float scal, LpMatrix3x3f* src, LpMatrix3x3f* dest)
{
	dest->data[0][0] = src->data[0][0]*scal;
	dest->data[0][1] = src->data[0][1]*scal;
	dest->data[0][2] = src->data[0][2]*scal;

	dest->data[1][0] = src->data[1][0]*scal;
	dest->data[1][1] = src->data[1][1]*scal;
	dest->data[1][2] = src->data[1][2]*scal;

	dest->data[2][0] = src->data[2][0]*scal;
	dest->data[2][1] = src->data[2][1]*scal;
	dest->data[2][2] = src->data[2][2]*scal;

	return 1;
}

int scalarMatMult4x4(float scal, LpMatrix4x4f* src, LpMatrix4x4f* dest)
{
	dest->data[0][0] = src->data[0][0]*scal;
	dest->data[0][1] = src->data[0][1]*scal;
	dest->data[0][2] = src->data[0][2]*scal;
	dest->data[0][3] = src->data[0][3]*scal;

	dest->data[1][0] = src->data[1][0]*scal;
	dest->data[1][1] = src->data[1][1]*scal;
	dest->data[1][2] = src->data[1][2]*scal;
	dest->data[1][3] = src->data[1][3]*scal;

	dest->data[2][0] = src->data[2][0]*scal;
	dest->data[2][1] = src->data[2][1]*scal;
	dest->data[2][2] = src->data[2][2]*scal;
	dest->data[2][3] = src->data[2][3]*scal;

	dest->data[3][0] = src->data[3][0]*scal;
	dest->data[3][1] = src->data[3][1]*scal;
	dest->data[3][2] = src->data[3][2]*scal;
	dest->data[3][3] = src->data[3][3]*scal;

	return 1;
}

int scalarMatMult3x4(float scal, LpMatrix3x4f* src, LpMatrix3x4f* dest)
{
	dest->data[0][0] = src->data[0][0]*scal;
	dest->data[0][1] = src->data[0][1]*scal;
	dest->data[0][2] = src->data[0][2]*scal;
	dest->data[0][3] = src->data[0][3]*scal;

	dest->data[1][0] = src->data[1][0]*scal;
	dest->data[1][1] = src->data[1][1]*scal;
	dest->data[1][2] = src->data[1][2]*scal;
	dest->data[1][3] = src->data[1][3]*scal;

	dest->data[2][0] = src->data[2][0]*scal;
	dest->data[2][1] = src->data[2][1]*scal;
	dest->data[2][2] = src->data[2][2]*scal;
	dest->data[2][3] = src->data[2][3]*scal;

	return 1;
}

int scalarVectMult4x1(float scal, LpVector4f* src, LpVector4f* dest)
{
	dest->data[0] = src->data[0] * scal;
	dest->data[1] = src->data[1] * scal;
	dest->data[2] = src->data[2] * scal;
	dest->data[3] = src->data[3] * scal;

	return 1;
}

int scalarVectMult3x1(float scal, LpVector3f* src, LpVector3f* dest)
{
	dest->data[0] = src->data[0] * scal;
	dest->data[1] = src->data[1] * scal;
	dest->data[2] = src->data[2] * scal;

	return 1;
}

void createIdentity3x3(LpMatrix3x3f* dest)
{
	matZero3x3(dest);

	dest->data[0][0] = 1;
	dest->data[1][1] = 1;
	dest->data[2][2] = 1;
}

void createIdentity4x4(LpMatrix4x4f* dest)
{
	matZero4x4(dest);

	dest->data[0][0] = 1;
	dest->data[1][1] = 1;
	dest->data[2][2] = 1;
	dest->data[3][3] = 1;
}

void matZero3x3(LpMatrix3x3f* dest)
{
	dest->data[0][0] = 0;
	dest->data[0][1] = 0;
	dest->data[0][2] = 0;

	dest->data[1][0] = 0;
	dest->data[1][1] = 0;
	dest->data[1][2] = 0;

	dest->data[2][0] = 0;
	dest->data[2][1] = 0;
	dest->data[2][2] = 0;
}

void matZero3x4(LpMatrix3x4f* dest)
{
	dest->data[0][0] = 0;
	dest->data[0][1] = 0;
	dest->data[0][2] = 0;
	dest->data[0][3] = 0;

	dest->data[1][0] = 0;
	dest->data[1][1] = 0;
	dest->data[1][2] = 0;
	dest->data[1][3] = 0;

	dest->data[2][0] = 0;
	dest->data[2][1] = 0;
	dest->data[2][2] = 0;
	dest->data[2][3] = 0;
}

void matZero4x3(LpMatrix4x3f* dest)
{
	dest->data[0][0] = 0;
	dest->data[0][1] = 0;
	dest->data[0][2] = 0;

	dest->data[1][0] = 0;
	dest->data[1][1] = 0;
	dest->data[1][2] = 0;

	dest->data[2][0] = 0;
	dest->data[2][1] = 0;
	dest->data[2][2] = 0;

	dest->data[3][0] = 0;
	dest->data[3][1] = 0;
	dest->data[3][2] = 0;
}

void matZero4x4(LpMatrix4x4f* dest)
{
	dest->data[0][0] = 0;
	dest->data[0][1] = 0;
	dest->data[0][2] = 0;
	dest->data[0][3] = 0;

	dest->data[1][0] = 0;
	dest->data[1][1] = 0;
	dest->data[1][2] = 0;
	dest->data[1][3] = 0;

	dest->data[2][0] = 0;
	dest->data[2][1] = 0;
	dest->data[2][2] = 0;
	dest->data[2][3] = 0;

	dest->data[3][0] = 0;
	dest->data[3][1] = 0;
	dest->data[3][2] = 0;
	dest->data[3][3] = 0;
}

void vectZero3x1(LpVector3f* dest)
{
	dest->data[0] = 0;
	dest->data[1] = 0;
	dest->data[2] = 0;
}

void vectSub3x1(LpVector3f* src1, LpVector3f* src2, LpVector3f* dest)
{
	int i;
	LpVector3f result;

	result.data[0] = src1->data[0] - src2->data[0];
	result.data[1] = src1->data[1] - src2->data[1];
	result.data[2] = src1->data[2] - src2->data[2];

	for (i = 0; i < 3; i++) {
		dest->data[i] = result.data[i];
	}	
}

void vectAdd4x1(LpVector4f* src1, LpVector4f* src2, LpVector4f* dest)
{
	int i;
	LpVector4f result;

	result.data[0] = src1->data[0] + src2->data[0];
	result.data[1] = src1->data[1] + src2->data[1];
	result.data[2] = src1->data[2] + src2->data[2];
	result.data[3] = src1->data[3] + src2->data[3];

	for (i = 0; i < 4; i++) {
		dest->data[i] = result.data[i];
	}	
}

void vectAdd3x1(LpVector3f* src1, LpVector3f* src2, LpVector3f* dest)
{
	int i;
	LpVector3f result;

	result.data[0] = src1->data[0] + src2->data[0];
	result.data[1] = src1->data[1] + src2->data[1];
	result.data[2] = src1->data[2] + src2->data[2];

	for (i = 0; i < 3; i++) {
		dest->data[i] = result.data[i];
	}	
}

void vecCWiseDiv3(LpVector3f* src1, LpVector3f* src2, LpVector3f* dest)
{
	int i;
	LpVector3f result;

	result.data[0] = src1->data[0] / src2->data[0];
	result.data[1] = src1->data[1] / src2->data[1];
	result.data[2] = src1->data[2] / src2->data[2];

	for (i = 0; i < 3; i++) {
		dest->data[i] = result.data[i];
	}	
}

void vecCWiseMult3(LpVector3f* src1, LpVector3f* src2, LpVector3f* dest)
{
	int i;
	LpVector3f result;

	result.data[0] = src1->data[0] * src2->data[0];
	result.data[1] = src1->data[1] * src2->data[1];
	result.data[2] = src1->data[2] * src2->data[2];

	for (i = 0; i < 3; i++) {
		dest->data[i] = result.data[i];
	}	
}

void matCopy3x3(LpMatrix3x3f* src, LpMatrix3x3f* dest)
{
	dest->data[0][0] = src->data[0][0];
	dest->data[0][1] = src->data[0][1];
	dest->data[0][2] = src->data[0][2];

	dest->data[1][0] = src->data[1][0];
	dest->data[1][1] = src->data[1][1];
	dest->data[1][2] = src->data[1][2];

	dest->data[2][0] = src->data[2][0];
	dest->data[2][1] = src->data[2][1];
	dest->data[2][2] = src->data[2][2];
}

float vect4x1Norm(LpVector4f src)
{
	// float f = 1.0f / sqrtf(src.data[0]*src.data[0] + src.data[1]*src.data[1] + src.data[2]*src.data[2] + src.data[3]*src.data[3]);

	float f = invSqrt(src.data[0]*src.data[0] + src.data[1]*src.data[1] + src.data[2]*src.data[2] + src.data[3]*src.data[3]);
	
	return f;
}

float vect3x1Norm(LpVector3f src)
{
	// float f = 1.0f / sqrtf(src.data[0]*src.data[0] + src.data[1]*src.data[1] + src.data[2]*src.data[2]);

	float f = invSqrt(src.data[0]*src.data[0] + src.data[1]*src.data[1] + src.data[2]*src.data[2]);
	
	return f;
}

void vect3x1SetScalar(LpVector3f* src, float scalar)
{
	src->data[0] = scalar;
	src->data[1] = scalar;
	src->data[2] = scalar;
}

void quaternionInv(LpVector4f* src, LpVector4f* dest)
{
	float n;
	
	dest->data[0] = src->data[0];
	dest->data[1] = -src->data[1];
	dest->data[2] = -src->data[2];
	dest->data[3] = -src->data[3];	
	
	n = vect4x1Norm(*dest);
	scalarVectMult4x1(n, dest, dest);
}

void quaternionMult(LpVector4f* src1, LpVector4f* src2, LpVector4f* dest)
{
	float a, b, c, d;
	float e, f, g, h;
	
	a = src1->data[0];
	b = src1->data[1];
	c = src1->data[2];
	d = src1->data[3];
	
	e = src2->data[0];
	f = src2->data[1];	 
	g = src2->data[2];
	h = src2->data[3];	
	
	dest->data[0] = a*e - b*f - c*g - d*h;
	dest->data[1] = b*e + a*f + c*h - d*g;
	dest->data[2] = a*g - b*h + c*e + d*f;
	dest->data[3] = a*h + b*g - c*f + d*e;	
}

void quaternionCon(LpVector4f* src, LpVector4f* dest)
{
	dest->data[0] = src->data[0];
	dest->data[1] = -src->data[1];
	dest->data[2] = -src->data[2];
	dest->data[3] = -src->data[3];	
}

void quatRotVec(LpVector4f q, LpVector3f vI, LpVector3f* vO)
{
	LpVector4f tQ;
	LpVector4f tQ2;
	LpVector4f tQ3;	
	LpVector4f tQ4;	
	
	tQ.data[0] = 0.0f;
	tQ.data[1] = vI.data[0];
	tQ.data[2] = vI.data[1];
	tQ.data[3] = vI.data[2];

	quaternionCon(&q, &tQ2);
	
	quaternionMult(&q, &tQ, &tQ3);
	quaternionMult(&tQ3, &tQ2, &tQ4);

	vO->data[0] = tQ4.data[1];
	vO->data[1] = tQ4.data[2];
	vO->data[2] = tQ4.data[3];
}

void quaternionToEuler(LpVector4f *q, LpVector3f *r)
{
	LpMatrix3x3f tM;
	double theta;
	double cp;
	float sp;

	quaternionToMatrix(q, &tM);

	sp = tM.data[0][2];
	if (sp > 1.0f) {
		sp = 1.0f;
	} else if (sp < -1.0f) {
		sp = -1.0f;
	}

	// theta = fxpt_atan2((int16_t)(sp*10000.0f), (int16_t)(sqrt2((1.0 + sp)*(1.0-sp))*10000.0f))/(float)0x8000 * M_1_PI * 4;
	
	theta = -asin(sp);
	cp = fastCos(theta);

	if (cp > (8192.0f * FLT_EPSILON)) {
		r->data[1] = theta;

		// r->data[2] = (float)(fxpt_atan2((int16_t)(tM.data[0][1]*10000.0f), (int16_t)(tM.data[0][0]*10000.0f)))/(float)0x8000 * M_1_PI * 4;
		// r->data[0] = (float)(fxpt_atan2((int16_t)(tM.data[1][2]*10000.0f), (int16_t)(tM.data[2][2]*10000.0f)))/(float)0x8000 * M_1_PI * 4;

		r->data[2] = atan2(tM.data[0][1], tM.data[0][0]);
		r->data[0] = atan2(tM.data[1][2], tM.data[2][2]);
	} else {
		r->data[1] = theta;

		// r->data[2] = (float) -fxpt_atan2((int16_t)(tM.data[1][0]*10000.0f), (int16_t)(tM.data[1][1]*10000.0f))/(float)0x8000 * M_1_PI * 4;

		r->data[2] = -atan2(tM.data[1][0], tM.data[1][1]);
		r->data[0] = 0;
	}
}

void quaternionToMatrix(LpVector4f *q, LpMatrix3x3f* M)
{	
	float tmp1;
	float tmp2;

	float sqw = q->data[0] * q->data[0];
	float sqx = q->data[1] * q->data[1];
	float sqy = q->data[2] * q->data[2];
	float sqz = q->data[3] * q->data[3];

	float invs = 1 / (sqx + sqy + sqz + sqw);
    
	M->data[0][0] = ( sqx - sqy - sqz + sqw) * invs;
	M->data[1][1] = (-sqx + sqy - sqz + sqw) * invs;
	M->data[2][2] = (-sqx - sqy + sqz + sqw) * invs;
	
	tmp1 = q->data[1] * q->data[2];
	tmp2 = q->data[3] * q->data[0];
	
	M->data[1][0] = 2.0f * (tmp1 + tmp2) * invs;
	M->data[0][1] = 2.0f * (tmp1 - tmp2) * invs;
    
	tmp1 = q->data[1] * q->data[3];
	tmp2 = q->data[2] * q->data[0];
	
	M->data[2][0] = 2.0f * (tmp1 - tmp2) * invs;
	M->data[0][2] = 2.0f * (tmp1 + tmp2) * invs;
    
	tmp1 = q->data[2] * q->data[3];
	tmp2 = q->data[1] * q->data[0];
    
	M->data[2][1] = 2.0f * (tmp1 + tmp2) * invs;
	M->data[1][2] = 2.0f * (tmp1 - tmp2) * invs;
}

/* void quaternionToMatrix(LpVector4f *q, LpMatrix3x3f* M)
{
	LpMatrix3x3f tM;
	
	float wx, wy, wz;
	float xx, yy, yz;
	float xy, xz, zz;
	float x2, y2, z2;

	x2 = q->data[1] + q->data[1];
	y2 = q->data[2] + q->data[2];
	z2 = q->data[3] + q->data[3];

	xx = q->data[1] * x2;
	xy = q->data[1] * y2;
	xz = q->data[1] * z2;

	yy = q->data[2] * y2;
	yz = q->data[2] * z2;
	zz = q->data[3] * z2;

	wx = q->data[0] * x2;
	wy = q->data[0] * y2;
	wz = q->data[0] * z2;

	tM.data[0][0] = 1.0f - (yy + zz);
	tM.data[0][1] = xy - wz;
	tM.data[0][2] = xz + wy;
	tM.data[1][0] = xy + wz;
	tM.data[1][1] = 1.0f - (xx + zz);
	tM.data[1][2] = yz - wx;
	tM.data[2][0] = xz - wy;
	tM.data[2][1] = yz + wx;
	tM.data[2][2] = 1.0f - (xx + yy);

	matTrans3x3(&tM, M); 
} */

/* void quaternionToMatrix(LpVector4f *q, LpMatrix3x3f* M)
{
	float qw = q->data[0];
	float qx = q->data[1];
	float qy = q->data[2];
	float qz = q->data[3];	
	
	M->data[0][0] = qw*qw + qx*qx - qy*qy - qz*qz;
	M->data[0][1] = 2*qx*qy + 2*qw*qz;
	M->data[0][2] = 2*qx*qz - 2*qw*qy;

	M->data[1][0] = 2*qx*qy - 2*qw*qz;
	M->data[1][1] = qw*qw - qx*qx + qy*qy - qz*qz;
	M->data[1][2] = 2*qy*qz + 2*qw*qx;

	M->data[2][0] = 2*qx*qz + 2*qw*qy;
	M->data[2][1] = 2*qy*qz - 2*qw*qx;
	M->data[2][2] = qw*qw - qx*qx - qy*qy + qz*qz;
} */

void convertLpMatrixToArray(LpMatrix3x3f* src, float dest[3][3])
{
	int i, j;

	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			dest[i][j] = src->data[i][j];
		}
	}
}

void convertLpVectorToArray(LpVector3f* src, float dest[3])
{
	int i;

	for (i=0; i<3; i++) {
		dest[i] = src->data[i];
	}
}

void quaternionIdentity(LpVector4f* dest) 
{
	dest->data[0] = 1.0f;
	dest->data[1] = 0.0f;
	dest->data[2] = 0.0f;
	dest->data[3] = 0.0f;
}