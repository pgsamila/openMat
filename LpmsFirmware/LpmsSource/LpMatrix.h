/***********************************************************************
** Matrix and vector operations
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
**
** Note: Parts of this code have been adapted from CH-Robotics 
** source code available at http://www.chrobotics.com
***********************************************************************/

#ifndef LP_MATRIX
#define LP_MATRIX

#include <math.h>

#ifdef __IAR_SYSTEMS_ICC__
	#include <stdint.h>

	typedef union _float2int {
		uint32_t u32_val;
		float float_val;
	} float2int;
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Type definitions
typedef struct _LpMatrix3x3f {
	float data[3][3];
} LpMatrix3x3f;

typedef struct _LpMatrix3x4f {
	float data[3][4];
} LpMatrix3x4f;

typedef struct _LpMatrix4x3f {
	float data[4][3];
} LpMatrix4x3f;

typedef struct _LpMatrix4x4f {
	float data[4][4];
} LpMatrix4x4f;

typedef struct _LpVector3f {
	float data[3];
} LpVector3f;

typedef struct _LpVector4f {
	float data[4];
} LpVector4f;

#ifdef __IAR_SYSTEMS_ICC__
	typedef struct _LpVector3i {
		int32_t data[3];
	} LpVector3i;

	typedef struct _LpVector3i32 {
		int32_t data[3];
	} LpVector3i32;
	
	typedef struct _LpVector4i {
		int16_t data[4];
	} LpVector4i;

	typedef struct _LpMatrix4x4i {
		uint32_t data[4][4];
	} LpMatrix4x4i;
#endif

float fastSqrt(const float x);
float fastSin(float x);
float fastCos(float x);

// Matrix manipulation functions
float matDet3x3(LpMatrix3x3f* src);
float matInv3x3(LpMatrix3x3f* src, LpMatrix3x3f* dest);
int matAdd3x3(LpMatrix3x3f* src1, LpMatrix3x3f* src2, LpMatrix3x3f* dest);
int matAdd4x4(LpMatrix4x4f* src1, LpMatrix4x4f* src2, LpMatrix4x4f* dest);
int matMult3x3(LpMatrix3x3f* src1, LpMatrix3x3f* src2, LpMatrix3x3f* dest);
int matMult3x4to4x4(LpMatrix3x4f* src1, LpMatrix4x4f* src2, LpMatrix3x4f* dest);
int matMult3x4to4x3(LpMatrix3x4f* src1, LpMatrix4x3f* src2, LpMatrix3x3f* dest);
int matMult4x4to4x3(LpMatrix4x4f* src1, LpMatrix4x3f* src2, LpMatrix4x3f* dest);
int matMult4x3to3x3(LpMatrix4x3f* src1, LpMatrix3x3f* src2, LpMatrix4x3f* dest);
int matMult4x3to3x4(LpMatrix4x3f* src1, LpMatrix3x4f* src2, LpMatrix4x4f* dest);
int matMult4x4(LpMatrix4x4f* src1, LpMatrix4x4f* src2, LpMatrix4x4f* dest);
int matVectMult3(LpMatrix3x3f* matrix, LpVector3f* vector, LpVector3f* dest);
int matVectMult4(LpMatrix4x4f* matrix, LpVector4f* vector, LpVector4f* dest);
int matVectMult3x4(LpMatrix3x4f* matrix, LpVector4f* vector, LpVector3f* dest);
int matVectMult4x3(LpMatrix4x3f* matrix, LpVector3f* vector, LpVector4f* dest);
int matTrans3x3(LpMatrix3x3f* src, LpMatrix3x3f* dest);
int matTrans4x4(LpMatrix4x4f* src, LpMatrix4x4f* dest);
int matTrans3x4(LpMatrix3x4f* src, LpMatrix4x3f* dest);
int scalarMatMult3x3(float scal, LpMatrix3x3f* src, LpMatrix3x3f* dest);
int scalarMatMult4x4(float scal, LpMatrix4x4f* src, LpMatrix4x4f* dest);
int scalarMatMult3x4(float scal, LpMatrix3x4f* src, LpMatrix3x4f* dest);
void createIdentity3x3(LpMatrix3x3f* dest);
void createIdentity4x4(LpMatrix4x4f* dest);
void matZero3x3(LpMatrix3x3f* dest);
void matZero4x4(LpMatrix4x4f* dest);
void matZero3x4(LpMatrix3x4f* dest);
void matZero4x3(LpMatrix4x3f* dest);
void matCopy3x3(LpMatrix3x3f* src, LpMatrix3x3f* dest);

// Vector manipulation functions
float vect3x1Norm(LpVector3f src);
float vect4x1Norm(LpVector4f src);
void vectZero3x1(LpVector3f* dest);
int scalarVectMult3x1(float scal, LpVector3f* src, LpVector3f* dest);
int scalarVectMult4x1(float scal, LpVector4f* src, LpVector4f* dest);
void vectAdd4x1(LpVector4f* src1, LpVector4f* src2, LpVector4f* dest);
void vectAdd3x1(LpVector3f* src1, LpVector3f* src2, LpVector3f* dest);
void vectSub3x1(LpVector3f* src1, LpVector3f* src2, LpVector3f* dest);
void vecCWiseDiv3(LpVector3f* src1, LpVector3f* src2, LpVector3f* dest);
void vecCWiseMult3(LpVector3f* src1, LpVector3f* src2, LpVector3f* dest);
void vect3x1SetScalar(LpVector3f* src, float scalar);

// Quaternion operations
void quaternionInv(LpVector4f* src, LpVector4f* dest);
void quaternionMult(LpVector4f* src1, LpVector4f* src2, LpVector4f* dest);
void quaternionToEuler(LpVector4f *q, LpVector3f *r);
void quaternionToMatrix(LpVector4f *q, LpMatrix3x3f *M);
void quaternionIdentity(LpVector4f* dest);
void quaternionCon(LpVector4f* src, LpVector4f* dest);
void quatRotVec(LpVector4f q, LpVector3f vI, LpVector3f* vO);

// Conversions
void convertLpMatrixToArray(LpMatrix3x3f* src, float dest[3][3]);
void convertLpVectorToArray(LpVector3f* src, float dest[3]);

#ifdef __cplusplus
}
#endif 

#endif
