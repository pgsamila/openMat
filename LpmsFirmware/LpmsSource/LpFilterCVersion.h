/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#ifndef LP_FILTER_C_VERSION
#define LP_FILTER_C_VERSION

#define GYR_TEMP_TABLE_MAX 16
#define GYR_CAL_MIN_TEMP_DIFF 5.0f

#define GYRO_ONLINE_CAL_ITER 32
#define GYR_CAL_THRES 60.0f
#define GYR_CAL_TIMEOUT	7.5f
#define GYR_THRESHOLD 0.05f

#define LPMS_FILTER_GYR 0
#define LPMS_FILTER_GYR_ACC 1
#define LPMS_FILTER_GYR_ACC_MAG 2
#define LPMS_FILTER_ACC_MAG 3
#define LPMS_FILTER_GYR_ACC_EULER 4

#define LPMS_LIN_ACC_COMP_MODE_OFF 0
#define LPMS_LIN_ACC_COMP_MODE_WEAK 1
#define LPMS_LIN_ACC_COMP_MODE_MEDIUM 2
#define LPMS_LIN_ACC_COMP_MODE_STRONG 3
#define LPMS_LIN_ACC_COMP_MODE_ULTRA 4

#define LPMS_CENTRI_COMP_MODE_OFF 0
#define LPMS_CENTRI_COMP_MODE_ON 1

#define LPMS_OFFSET_MODE_OBJECT 0
#define LPMS_OFFSET_MODE_HEADING 1
#define LPMS_OFFSET_MODE_ALIGNMENT 2

#define LPMS_DEFAULT_ACC_COVAR 0.01f
#define LPMS_DEFAULT_MAG_COVAR 0.5f
#define LPMS_DEFAULT_PROCESS_COVAR 5.0e-5f

#include "LpMatrix.h"

#ifdef __cplusplus
extern "C" {
#endif 

typedef struct _LpmsCalibrationData {
	LpVector3f gyrOffset;
	LpVector3f gyrGain;     
	uint32_t gyrRange;
	uint32_t gyrOutputRate;
	LpMatrix3x3f gyrAlignment;
	LpVector3f gyrAlignOffset;

	LpVector3f accGain;
	LpVector3f accOffset;
	uint32_t accRange;
	uint32_t accOutputRate;
	LpMatrix3x3f accAlignment;

	LpVector3f magGain;
	LpVector3f magOffset;
	LpMatrix3x3f magSoftIronMatrix;
	LpMatrix3x3f magAlignmentMatrix;
	LpVector3f magAlignmentOffset;	
	uint32_t magRange;
	uint32_t magOutputRate;
	
	LpVector3f gyrTempCalPrmA;
	LpVector3f gyrTempCalPrmB;
	LpVector3f gyrTempCalBaseV;
	float gyrTempCalBaseT;

	float lpAlpha;

	uint32_t canMapping[16];
	float canHeartbeatTiming;
} LpmsCalibrationData;

typedef struct _LpFilterParameters { 
	LpVector3f accRef;
	LpVector3f magRef;

	LpVector3f gyrThreshold;
	LpVector3f magThreshold;

	float accCovar;
	float magCovar;

	float magFieldEstimate;
	float magInclination;
              
	uint32_t dynamicCovar;

	uint32_t useGyrThreshold;
	uint32_t useGyrAutoCal;

	uint32_t filterMode;
	uint32_t linAccCompMode;
	uint32_t centriCompMode;
	
	LpVector4f offsetQ;
} LpFilterParameters;

typedef struct _gtTableEntry {
	LpVector3f offset;
	float temp;
	LpVector3f m;
	float t0;
	LpVector3f o0;
} gtTableEntry;

void lpFilterInit(void);

void lpFilterUpdate(LpVector3f a, 
	LpVector3f b, 
	LpVector3f g, 
	LpVector4f *qOut, 
	float T, 
	float bInclination, 
	float *bNoise, 
	LpmsCalibrationData *calibrationData, 
	LpFilterParameters *lpFilterParam);

void lpFilterEulerUpdate(
	LpVector3f a, 
	LpVector3f b, 
	LpVector3f g, 
	LpVector3f *rOut, 
	LpVector4f *qOut, 
	float T, 
	float bInclination, 
	float *bNoise,
	LpmsCalibrationData *calibrationData, 
	LpFilterParameters *lpFilterParam);


void lpOrientationFromAccMag(LpVector3f b,
	LpVector3f a,
	LpVector3f *r,
	float *i,
	LpmsCalibrationData *calibrationData,
	LpFilterParameters *lpFilterParam);

void gyrOnlineCal(LpVector3f gyrRawData, 
	float T,
	int isGyrCalibrationEnabled,
	LpmsCalibrationData *calibrationData, 
	LpFilterParameters *lpFilterParam);

void gyroToInertial(LpVector4f qI, 
	LpVector3f *wO, 
	LpVector3f gRaw);

void gyroToInertialEuler(LpVector3f gRaw, 
	LpVector3f rAfterOffset,
	LpVector3f *w);

LpVector4f applyAlignmentOffset(LpVector4f q, 
	uint32_t mode, 
	LpVector4f *mQ_hx, 
	LpVector4f *mQ_offset);

void calculateAlignmentOffset(LpVector4f q, 
	LpVector4f *mQ_hx, 
	LpVector4f *mQ_offset);

#ifdef __cplusplus
}
#endif 
	
#endif