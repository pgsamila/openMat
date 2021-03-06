/***********************************************************************
** (C) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "SensorManager.h"

LpVector3f gyrRawData;
LpVector3f accRawData;
LpVector3f magRawData;
int16_t rawTemp;
float temperature;
float temperatureBias;
int32_t rawPressure;
float pressure;
float pressureBias;
float altitude;
LpVector3f a;
LpVector3f aRaw;
LpVector3f b;
LpVector3f bRaw;
LpVector3f g; 
LpVector3f gRaw; 
LpVector4f q;
LpVector4f qOffset;
LpVector4f qAfterOffset;
LpVector3f r;
LpVector3f w;
LpVector3f rAfterOffset;
LpVector3f linAcc;
LpVector3f translation;
LpVector3f velocity;
float magNoise = 0.0f;
int8_t gyroTempData;
int16_t accMagTempData;
int16_t tempData;
int32_t pressureData;
uint8_t currentMode;
LpmsCalibrationData calibrationData; 
LpFilterParameters lpFilterParam;
uint8_t isGyrCalibrationEnabled = 0;
float cumulatedGyrData[3] = { 0, 0, 0 };
float startGyrData[3] = { 0, 0, 0 };
float endGyrData[3] = { 0, 0, 0 };
LpVector3f gDrift;
uint32_t cumulatedGyrCounter = 0;
uint32_t startGyrCounter = 0;
uint32_t endGyrCounter = 0;
float gyrCalibrationDuration = 0;
uint8_t isRefCalibrationEnabled = 0;
float cumulatedRefData[3] = { 0.0f, 0.0f, 0.0f };
int32_t cumulatedRefCounter = 0;
float refCalibrationDuration = 0;
LpVector3f maxGyr;	
LpVector3f minGyr;
uint32_t lpmsStatus = 0;
static __IO uint8_t isDataSending = 0;
uint16_t pressureTime = 0;
float canHeartbeatTime = 0.0f;
float gLowX = 0.0f;
float gLowY = 0.0f;
float gLowZ = 0.0f;
float preGX = 0.0f;
float preGY = 0.0f;
float preGZ = 0.0f;
float aAvg[3];
int32_t preAvg[3];
float startTime = 0.0f;
int32_t pCycleCount = 0;
LpVector3f gyrRawDataLp;
LpVector3f accRawDataLp;
LpVector3f magRawDataLp;
LpVector3f tB;
static float d2r = 0.01745f;
uint32_t measurementTime = 0;
float sendTime = 0.0f;
LpVector3f heaveOutput;
float heaveY = 0.0f;
LpVector3f aRawNoLp;
LpVector4f mQ_hx;
LpVector4f mQ_offset;
float lpmsMeasurementPeriod = 0.00250f;
__IO uint8_t lpmsMeasurementIntervals = 2;
uint16_t lpmsLedPeriod = 400;
uint16_t bmp180MeasurePeriod = 50;  // 125ms x 4, see routine for more details
uint8_t bmp180MeasurementMode = BMP180_HIGH_MODE;

#ifdef ENABLE_INSOLE
	float forceSensorOutput[N_FORCE_SENSORS];
#endif

extern uint8_t isSelfTestOn;
extern LpmsReg gReg;
extern uint8_t isFirmwareUpdating;
extern uint8_t connectedInterface;
extern uint8_t isTimestampResetArmed;
extern uint16_t ledFlashTime;
extern uint16_t ledC;

void initSensorManager(void)
{
  	uint32_t buffer, address;
	
	if (CHECK_USER_FLASH()) {
		buffer = (uint32_t)gReg.data;
		address = USER_FLASH_START_ADDRESS;
		loadFlashToRam(&address, (uint32_t*)buffer, REG_ARRAY_SIZE);
	} else {
		if (resetToFactory() == 0) {
			lpmsStatus = lpmsStatus | LPMS_FLASH_WRITE_FAILED;
			return;
		}
		buffer = (uint32_t)gReg.data;
		address = USER_FLASH_START_ADDRESS;
		loadFlashToRam(&address, (uint32_t*)buffer, REG_ARRAY_SIZE);
	}
	
	float2int f2int;
	f2int.u32_val = gReg.data[LPMS_GYR_GAIN_X];
	calibrationData.gyrGain.data[0] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_GYR_GAIN_Y];
	calibrationData.gyrGain.data[1] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_GYR_GAIN_Z];
	calibrationData.gyrGain.data[2] = f2int.float_val;

	f2int.u32_val = gReg.data[LPMS_GYR_BIAS_X];
	calibrationData.gyrOffset.data[0] = f2int.float_val;  
	f2int.u32_val = gReg.data[LPMS_GYR_BIAS_Y];
	calibrationData.gyrOffset.data[1] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_GYR_BIAS_Z];
	calibrationData.gyrOffset.data[2] = f2int.float_val;
	
	calibrationData.gyrAlignOffset.data[0] = conItoF(gReg.data[LPMS_GYR_ALIG_BIAS_X]);
	calibrationData.gyrAlignOffset.data[1] = conItoF(gReg.data[LPMS_GYR_ALIG_BIAS_Y]);
	calibrationData.gyrAlignOffset.data[2] = conItoF(gReg.data[LPMS_GYR_ALIG_BIAS_Z]);

	calibrationData.gyrAlignment.data[0][0] = conItoF(gReg.data[LPMS_GYR_ALIG_00]);
	calibrationData.gyrAlignment.data[0][1] = conItoF(gReg.data[LPMS_GYR_ALIG_01]);
	calibrationData.gyrAlignment.data[0][2] = conItoF(gReg.data[LPMS_GYR_ALIG_02]);
	calibrationData.gyrAlignment.data[1][0] = conItoF(gReg.data[LPMS_GYR_ALIG_10]);
	calibrationData.gyrAlignment.data[1][1] = conItoF(gReg.data[LPMS_GYR_ALIG_11]);
	calibrationData.gyrAlignment.data[1][2] = conItoF(gReg.data[LPMS_GYR_ALIG_12]);
	calibrationData.gyrAlignment.data[2][0] = conItoF(gReg.data[LPMS_GYR_ALIG_20]);
	calibrationData.gyrAlignment.data[2][1] = conItoF(gReg.data[LPMS_GYR_ALIG_21]);
	calibrationData.gyrAlignment.data[2][2] = conItoF(gReg.data[LPMS_GYR_ALIG_22]);
	
	f2int.u32_val = gReg.data[LPMS_GYR_THRES_X];
	lpFilterParam.gyrThreshold.data[0] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_GYR_THRES_Y];
	lpFilterParam.gyrThreshold.data[1] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_GYR_THRES_Z];
	lpFilterParam.gyrThreshold.data[2] = f2int.float_val;
	
	if ((gReg.data[LPMS_CONFIG] & LPMS_GYR_THRES_ENABLED) != 0) {
		lpFilterParam.useGyrThreshold = 1;
	} else {
		lpFilterParam.useGyrThreshold = 0;
	}
	
	calibrationData.gyrRange = gReg.data[LPMS_GYR_RANGE];	

	if ((gReg.data[LPMS_CONFIG] & LPMS_GYR_AUTOCAL_ENABLED) != 0) {
		lpFilterParam.useGyrAutoCal = 1;
	} else {
		lpFilterParam.useGyrAutoCal = 0;
	}
	
	if (!initGyr(gReg.data[LPMS_GYR_OUTPUT_RATE], GYR_NORMAL_MODE, gReg.data[LPMS_GYR_RANGE])) lpmsStatus = lpmsStatus | LPMS_GYR_INIT_FAILED;
	
	f2int.u32_val = gReg.data[LPMS_ACC_GAIN_X];
	calibrationData.accGain.data[0] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_ACC_GAIN_Y];
	calibrationData.accGain.data[1] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_ACC_GAIN_Z];
	calibrationData.accGain.data[2] = f2int.float_val;
	
	calibrationData.accOffset.data[0] = conItoF(gReg.data[LPMS_ACC_BIAS_X]);
	calibrationData.accOffset.data[1] = conItoF(gReg.data[LPMS_ACC_BIAS_Y]);
	calibrationData.accOffset.data[2] = conItoF(gReg.data[LPMS_ACC_BIAS_Z]);

	f2int.u32_val = gReg.data[LPMS_ACC_COVAR_USER];
	lpFilterParam.accCovar = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_MAG_COVAR_USER];
	lpFilterParam.magCovar = f2int.float_val;

	f2int.u32_val = gReg.data[LPMS_ACC_REF_X];
	lpFilterParam.accRef.data[0] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_ACC_REF_Y];
	lpFilterParam.accRef.data[1] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_ACC_REF_Z];
	lpFilterParam.accRef.data[2] = f2int.float_val;
	
	calibrationData.accAlignment.data[0][0] = conItoF(gReg.data[LPMS_ACC_ALIG_00]);
	calibrationData.accAlignment.data[0][1] = conItoF(gReg.data[LPMS_ACC_ALIG_01]);
	calibrationData.accAlignment.data[0][2] = conItoF(gReg.data[LPMS_ACC_ALIG_02]);
	calibrationData.accAlignment.data[1][0] = conItoF(gReg.data[LPMS_ACC_ALIG_10]);
	calibrationData.accAlignment.data[1][1] = conItoF(gReg.data[LPMS_ACC_ALIG_11]);
	calibrationData.accAlignment.data[1][2] = conItoF(gReg.data[LPMS_ACC_ALIG_12]);
	calibrationData.accAlignment.data[2][0] = conItoF(gReg.data[LPMS_ACC_ALIG_20]);
	calibrationData.accAlignment.data[2][1] = conItoF(gReg.data[LPMS_ACC_ALIG_21]);
	calibrationData.accAlignment.data[2][2] = conItoF(gReg.data[LPMS_ACC_ALIG_22]);
	
	calibrationData.accRange = gReg.data[LPMS_ACC_RANGE];
	
	if (!initAcc(gReg.data[LPMS_ACC_OUTPUT_RATE], ACC_NORMAL_POWER_MODE, gReg.data[LPMS_ACC_RANGE])) lpmsStatus = lpmsStatus | LPMS_ACC_INIT_FAILED;

	f2int.u32_val = gReg.data[LPMS_MAG_GAIN_X];
	calibrationData.magGain.data[0] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_MAG_GAIN_Y];
	calibrationData.magGain.data[1] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_MAG_GAIN_Z];
	calibrationData.magGain.data[2] = f2int.float_val; 

	f2int.u32_val = gReg.data[LPMS_MAG_BIAS_X];
	calibrationData.magOffset.data[0] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_MAG_BIAS_Y];	
	calibrationData.magOffset.data[1] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_MAG_BIAS_Z];
	calibrationData.magOffset.data[2] = f2int.float_val;

	calibrationData.magSoftIronMatrix.data[0][0] = conItoF(gReg.data[LPMS_MAG_SOFT_00]);
	calibrationData.magSoftIronMatrix.data[0][1] = conItoF(gReg.data[LPMS_MAG_SOFT_01]);
	calibrationData.magSoftIronMatrix.data[0][2] = conItoF(gReg.data[LPMS_MAG_SOFT_02]);
	calibrationData.magSoftIronMatrix.data[1][0] = conItoF(gReg.data[LPMS_MAG_SOFT_10]);
	calibrationData.magSoftIronMatrix.data[1][1] = conItoF(gReg.data[LPMS_MAG_SOFT_11]);
	calibrationData.magSoftIronMatrix.data[1][2] = conItoF(gReg.data[LPMS_MAG_SOFT_12]);
	calibrationData.magSoftIronMatrix.data[2][0] = conItoF(gReg.data[LPMS_MAG_SOFT_20]);
	calibrationData.magSoftIronMatrix.data[2][1] = conItoF(gReg.data[LPMS_MAG_SOFT_21]);
	calibrationData.magSoftIronMatrix.data[2][2] = conItoF(gReg.data[LPMS_MAG_SOFT_22]);

	f2int.u32_val = gReg.data[LPMS_MAG_REF_X];
	lpFilterParam.magRef.data[0] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_MAG_REF_Y];
	lpFilterParam.magRef.data[1] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_MAG_REF_Z];
	lpFilterParam.magRef.data[2] = f2int.float_val;
	
	f2int.u32_val = gReg.data[LPMS_MAG_THRES_X];
	lpFilterParam.magThreshold.data[0] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_MAG_THRES_Y];
	lpFilterParam.magThreshold.data[1] = f2int.float_val;
	f2int.u32_val = gReg.data[LPMS_MAG_THRES_Z];
	lpFilterParam.magThreshold.data[2] = f2int.float_val;
	
	calibrationData.magRange = gReg.data[LPMS_MAG_RANGE];
	
	lpFilterParam.magInclination = conItoF(gReg.data[LPMS_MAG_FIELD_INC]);
	
	if (!initMag(gReg.data[LPMS_MAG_OUTPUT_RATE], MAG_NORMAL_POWER_MODE, gReg.data[LPMS_MAG_RANGE])) lpmsStatus = lpmsStatus | LPMS_MAG_INIT_FAILED;
	
	lpFilterParam.magFieldEstimate = conItoF(gReg.data[LPMS_MAG_FIELD_EST]);

	if ((gReg.data[LPMS_CONFIG] & LPMS_DYNAMIC_COVAR_ENABLED) != 0) {
		lpFilterParam.dynamicCovar = 1;
	} else {
		lpFilterParam.dynamicCovar = 0;
	}

	if (!initPressureSensor()) lpmsStatus = lpmsStatus | LPMS_PRESSURE_INIT_FAILED;
	            
    /*
	qOffset.data[0] = conItoF(gReg.data[LPMS_OFFSET_QUAT_0]);
	qOffset.data[1] = conItoF(gReg.data[LPMS_OFFSET_QUAT_1]);
	qOffset.data[2] = conItoF(gReg.data[LPMS_OFFSET_QUAT_2]);
	qOffset.data[3] = conItoF(gReg.data[LPMS_OFFSET_QUAT_3]);
    */
    mQ_offset.data[0] = conItoF(gReg.data[LPMS_OFFSET_QUAT_0]);
	mQ_offset.data[1] = conItoF(gReg.data[LPMS_OFFSET_QUAT_1]);
	mQ_offset.data[2] = conItoF(gReg.data[LPMS_OFFSET_QUAT_2]);
	mQ_offset.data[3] = conItoF(gReg.data[LPMS_OFFSET_QUAT_3]);
    
    mQ_hx.data[0] = conItoF(gReg.data[LPMS_HEADING_OFFSET_0]);
    mQ_hx.data[1] = conItoF(gReg.data[LPMS_HEADING_OFFSET_1]);
    mQ_hx.data[2] = conItoF(gReg.data[LPMS_HEADING_OFFSET_2]);
    mQ_hx.data[3] = conItoF(gReg.data[LPMS_HEADING_OFFSET_3]);
    
	qAfterOffset.data[0] = 1;
	qAfterOffset.data[1] = 0;
	qAfterOffset.data[2] = 0;
	qAfterOffset.data[3] = 0;
	
	vectZero3x1(&gRaw);
	vectZero3x1(&gDrift);

	lpFilterInit();

	initializeMCal();
	setHardIronOffset(calibrationData.magOffset);
	setSoftIronMatrix(calibrationData.magSoftIronMatrix); 

	vectZero3x1(&gyrRawDataLp);
	vectZero3x1(&accRawDataLp);
	vectZero3x1(&magRawDataLp);

	updateFilterMode();
	updateRawDataLp();
	updateCanMapping();
	updateCanHeartbeat();
	updateLinAccCompMode();
	updateCentriCompMode();
	updateMagAlignMatrix();
	updateMagAlignBias();
	updateMagReference();
	updateUartFormat();
	updateStreamFreq();

	//quaternionIdentity(&mQ_hx);
	//quaternionIdentity(&mQ_offset);
                    
#ifdef USE_HEAVEMOTION
	if ((gReg.data[LPMS_CONFIG] & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) initHeaveMotion();
#endif
}

void updateSensorData(void)
{     
	if (isSelfTestOn == 1) {
		calibrationData.gyrOffset.data[0] = 0.0f;
		calibrationData.gyrOffset.data[1] = 0.0f;
		calibrationData.gyrOffset.data[2] = 0.0f;
		
		/* horizontalRotationNext(&gyrRawData, &accRawData, &magRawData, 
			calibrationData.gyrGain, calibrationData.accGain, calibrationData.magGain, 
			lpFilterParam.accRef, lpFilterParam.magRef,
			T); */
	} else {
		getGyrRawData(&gyrRawData.data[0], &gyrRawData.data[1], &gyrRawData.data[2]);
		getAccRawData(&accRawData.data[0], &accRawData.data[1], &accRawData.data[2]);
		getMagRawData(&magRawData.data[0], &magRawData.data[1], &magRawData.data[2]);
#ifdef USE_CANBUS_INTERFACE
		canHeartbeatTime += lpmsMeasurementPeriod;
#endif

		++measurementTime;

#ifdef ENABLE_PRESSURE
		++pressureTime;

        if (pressureTime >= bmp180MeasurePeriod ){
            // note: bmp180 takes 4 cycles to complete one measurement
            // pressure/altitude update time = lpmsMeasurementPeriod x bmp180MeasurePeriod x 4 = 500ms
            pressureTime = 0;
			if (	(((gReg.data[LPMS_CONFIG] & LPMS_PRESSURE_OUTPUT_ENABLED) != 0) ||
				((gReg.data[LPMS_CONFIG] & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0) ||
				((gReg.data[LPMS_CONFIG] & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0)) && 
				((lpmsStatus & LPMS_PRESSURE_INIT_FAILED) != LPMS_PRESSURE_INIT_FAILED)) {
				  
				if ( getTempAndPressure(&rawTemp, &rawPressure, bmp180MeasurementMode) )
                      calcAltitude();
			}
		}
#endif
	}

	checkGyrCal();
	checkTimestampReset();

#ifdef ENABLE_INSOLE
	for (int i=0; i<N_FORCE_SENSORS; ++i) {
		forceSensorOutput[i] = getForceSensorChannel(i);
	}
#endif
}
  
void checkTimestampReset(void)
{
	if (isTimestampResetArmed == 1) {
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_13) == RESET) {
			measurementTime = 0;
			ledFlashTime = lpmsLedPeriod;
			isTimestampResetArmed = 0;
			ledC = 0;
			GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, (BitAction)0);
		}
	}
}

void processSensorData(void)
{     
	// float bInc;

	aRaw.data[0] = accRawData.data[0] * calibrationData.accGain.data[0];
	aRaw.data[1] = accRawData.data[1] * calibrationData.accGain.data[1];
	aRaw.data[2] = accRawData.data[2] * calibrationData.accGain.data[2];

	aRawNoLp = aRaw;

	gRaw.data[0] = (gyrRawData.data[0] - calibrationData.gyrOffset.data[0]) * calibrationData.gyrGain.data[0] * d2r;
	gRaw.data[1] = (gyrRawData.data[1] - calibrationData.gyrOffset.data[1]) * calibrationData.gyrGain.data[1] * d2r;
	gRaw.data[2] = (gyrRawData.data[2] - calibrationData.gyrOffset.data[2]) * calibrationData.gyrGain.data[2] * d2r;

	tB.data[0] = magRawData.data[0] * calibrationData.magGain.data[0];
	tB.data[1] = magRawData.data[1] * calibrationData.magGain.data[1];
	tB.data[2] = magRawData.data[2] * calibrationData.magGain.data[2];
 
	gyrOnlineCal(gyrRawData, lpmsMeasurementPeriod, isGyrCalibrationEnabled, &calibrationData, &lpFilterParam);

	applyLowPass();

	bRaw.data[0] = tB.data[0];
	bRaw.data[1] = tB.data[1];
	bRaw.data[2] = tB.data[2];

	b.data[0] = tB.data[0];
	b.data[1] = tB.data[1];
	b.data[2] = tB.data[2];

	b = correctB(b);

	matVectMult3(&calibrationData.accAlignment, &aRaw, &a);
	vectAdd3x1(&calibrationData.accOffset, &a, &a);

	matVectMult3(&calibrationData.gyrAlignment, &gRaw, &g);
	vectAdd3x1(&calibrationData.gyrAlignOffset, &g, &g);

	if (	lpFilterParam.filterMode == LPMS_FILTER_GYR ||
			lpFilterParam.filterMode == LPMS_FILTER_GYR_ACC || 
			lpFilterParam.filterMode == LPMS_FILTER_GYR_ACC_MAG) {

		lpFilterUpdate(a, b, g, &q, lpmsMeasurementPeriod, 0, &magNoise, &calibrationData, &lpFilterParam);
		
		qAfterOffset = applyAlignmentOffset(q, gReg.data[LPMS_OFFSET_MODE], &mQ_hx, &mQ_offset);

		if ((gReg.data[LPMS_CONFIG] & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) gyroToInertial(q, &w, gRaw);
		if ((gReg.data[LPMS_CONFIG] & LPMS_EULER_OUTPUT_ENABLED) != 0) quaternionToEuler(&qAfterOffset, &rAfterOffset);
		if ((gReg.data[LPMS_CONFIG] & LPMS_LINACC_OUTPUT_ENABLED) != 0) calcLinearAcceleration();
		
#ifdef USE_HEAVEMOTION
		if ((gReg.data[LPMS_CONFIG] & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) calculateHeaveMotion(lpmsMeasurementPeriod);
#endif

	} else if (lpFilterParam.filterMode == LPMS_FILTER_MADGWICK_GYR_ACC_MAG) {

	 	MadgwickAHRSupdate(a, b, g, lpmsMeasurementPeriod, &q);

		qAfterOffset = applyAlignmentOffset(q, gReg.data[LPMS_OFFSET_MODE], &mQ_hx, &mQ_offset);

		if ((gReg.data[LPMS_CONFIG] & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) gyroToInertial(q, &w, gRaw);
		if ((gReg.data[LPMS_CONFIG] & LPMS_EULER_OUTPUT_ENABLED) != 0) quaternionToEuler(&qAfterOffset, &rAfterOffset);
		if ((gReg.data[LPMS_CONFIG] & LPMS_LINACC_OUTPUT_ENABLED) != 0) calcLinearAcceleration();
		
#ifdef USE_HEAVEMOTION
		if ((gReg.data[LPMS_CONFIG] & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) calculateHeaveMotion(lpmsMeasurementPeriod);
#endif

	} /* else if (lpFilterParam.filterMode == LPMS_FILTER_GYR_ACC_EULER) {
		
		lpFilterEulerUpdate(aRaw, b, gRaw, &rAfterOffset, &q, lpmsMeasurementPeriod, 0, &magNoise, &calibrationData, &lpFilterParam);
		quaternionIdentity(&qAfterOffset); 
		if ((gReg.data[LPMS_CONFIG] & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) gyroToInertial(q, &w, gRaw);
		
	} else if (lpFilterParam.filterMode == LPMS_FILTER_ACC_MAG) {
	  
		lpOrientationFromAccMag(b, a, &rAfterOffset, &bInc, &calibrationData, &lpFilterParam);
		quaternionIdentity(&qAfterOffset);
		if ((gReg.data[LPMS_CONFIG] & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) gyroToInertialEuler(gRaw, rAfterOffset, &w);

	} */ else if (lpFilterParam.filterMode == LPMS_FILTER_MADGWICK_GYR_ACC) {

		vectZero3x1(&b);
	 	MadgwickAHRSupdate(a, b, g, lpmsMeasurementPeriod, &q);

		qAfterOffset = applyAlignmentOffset(q, gReg.data[LPMS_OFFSET_MODE], &mQ_hx, &mQ_offset);

		if ((gReg.data[LPMS_CONFIG] & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) gyroToInertial(q, &w, gRaw);
		if ((gReg.data[LPMS_CONFIG] & LPMS_EULER_OUTPUT_ENABLED) != 0) quaternionToEuler(&qAfterOffset, &rAfterOffset);
		if ((gReg.data[LPMS_CONFIG] & LPMS_LINACC_OUTPUT_ENABLED) != 0) calcLinearAcceleration();
		
#ifdef USE_HEAVEMOTION
		if ((gReg.data[LPMS_CONFIG] & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) calculateHeaveMotion(lpmsMeasurementPeriod);
#endif

	} else {

	 	MadgwickAHRSupdate(a, b, g, lpmsMeasurementPeriod, &q);

		qAfterOffset = applyAlignmentOffset(q, gReg.data[LPMS_OFFSET_MODE], &mQ_hx, &mQ_offset);

		if ((gReg.data[LPMS_CONFIG] & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) gyroToInertial(q, &w, gRaw);
		if ((gReg.data[LPMS_CONFIG] & LPMS_EULER_OUTPUT_ENABLED) != 0) quaternionToEuler(&qAfterOffset, &rAfterOffset);
		if ((gReg.data[LPMS_CONFIG] & LPMS_LINACC_OUTPUT_ENABLED) != 0) calcLinearAcceleration();
		
#ifdef USE_HEAVEMOTION
		if ((gReg.data[LPMS_CONFIG] & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) calculateHeaveMotion(lpmsMeasurementPeriod);
#endif

	}
}    

void applyLowPass(void)
{
	int i;

	if (calibrationData.lpAlpha > 0.0f) {
		for (i=0; i<3; ++i) {
			gyrRawDataLp.data[i] = 	gyrRawDataLp.data[i] * (1.0f - calibrationData.lpAlpha) + gRaw.data[i] * calibrationData.lpAlpha;
			gRaw.data[i] = gyrRawDataLp.data[i];
	
			accRawDataLp.data[i] = 	accRawDataLp.data[i] * (1.0f - calibrationData.lpAlpha) + aRaw.data[i] * calibrationData.lpAlpha;
			aRaw.data[i] = accRawDataLp.data[i];
	
			magRawDataLp.data[i] = 	magRawDataLp.data[i] * (1.0f - calibrationData.lpAlpha) + tB.data[i] * calibrationData.lpAlpha;
			tB.data[i] = magRawDataLp.data[i];
		}
	}
}

void calcAltitude(void)
{
	if (	(	((gReg.data[LPMS_CONFIG] & LPMS_PRESSURE_OUTPUT_ENABLED) != 0) ||
			((gReg.data[LPMS_CONFIG] & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0) ||
			((gReg.data[LPMS_CONFIG] & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0)) && 
			((lpmsStatus & LPMS_PRESSURE_INIT_FAILED) != LPMS_PRESSURE_INIT_FAILED)) {
			  
		pressure = (float) rawPressure * 1.0e-3f;
		temperature = (float) rawTemp * 1.0e-1f;

        	altitude = 44330.76067f * (1.0f - pow((pressure / 101.325), (1.0f / 5.25588f))); 
	}

}

void calcLinearAcceleration(void)
{
	LpVector3f gWorld;
	LpVector3f gSensor;
	LpMatrix3x3f M, M_i;

	gWorld.data[0] = 0;
	gWorld.data[1] = 0;
	gWorld.data[2] = -1;

#define LINACC_OUTPUT_LOCAL
#ifdef LINACC_OUTPUT_LOCAL
	quaternionToMatrix(&q, &M);
	matVectMult3(&M, &gWorld, &gSensor);
	vectSub3x1(&a, &gSensor, &linAcc);
#else
	quaternionToMatrix(&q, &M);
	matInv3x3(&M, &M_i);
	matVectMult3(&M_i, &a, &gSensor);
	vectSub3x1(&gSensor, &gWorld, &linAcc);
#endif
}

void accAverage(void)
{
	for (int i=0; i<3; i++) {
		aAvg[i] = 0.9 * aAvg[i] + 0.1 * a.data[i];
		a.data[i] = aAvg[i];
	}
}

void getFilterParam(LpFilterParameters *lpFP)
{
	*lpFP = lpFilterParam;
}

void setFilterParam(LpFilterParameters lpFP)
{
	lpFilterParam = lpFP;
}

void getSensorParam(LpmsCalibrationData *lCD)
{
	*lCD = calibrationData;
}

void setSensorParam(LpmsCalibrationData lCD)
{
	calibrationData = lCD;
}

uint8_t getCurrentMode(void)
{
	return currentMode;
}

void setCommandMode(void)
{
	currentMode = LPMS_COMMAND_MODE;
		
	lpmsStatus &= ~(LPMS_COMMAND_MODE | LPMS_STREAM_MODE | LPMS_SLEEP_MODE);
	lpmsStatus |= LPMS_COMMAND_MODE;

	isFirmwareUpdating = 0;
}

void setStreamMode(void)
{
	currentMode = LPMS_STREAM_MODE;
		
	lpmsStatus &= ~(LPMS_COMMAND_MODE | LPMS_STREAM_MODE | LPMS_SLEEP_MODE);
	lpmsStatus |= LPMS_STREAM_MODE;
}

void setSleepMode(void)
{
	currentMode = LPMS_SLEEP_MODE;
		
	lpmsStatus &= ~(LPMS_COMMAND_MODE | LPMS_STREAM_MODE | LPMS_SLEEP_MODE);
	lpmsStatus |= LPMS_SLEEP_MODE;
}

void startGyrCalibration(void)
{
  	if (isGyrCalibrationEnabled == 1) return;
  
	isGyrCalibrationEnabled = 1;
	
	maxGyr.data[0] = -1.0e4;
	maxGyr.data[1] = -1.0e4;
	maxGyr.data[2] = -1.0e4;

	minGyr.data[0] = 1.0e4;
	minGyr.data[1] = 1.0e4;
	minGyr.data[2] = 1.0e4;

	cumulatedGyrCounter = 0;
	startGyrCounter = 0;
	endGyrCounter = 0;

	for (uint8_t i = 0; i < 3; i++) cumulatedGyrData[i] = 0;
	for (uint8_t i = 0; i < 3; i++) startGyrData[i] = 0;
	for (uint8_t i = 0; i < 3; i++) endGyrData[i] = 0;
	
	lpmsStatus |= LPMS_GYR_CALIBRATION_RUNNING;	
}

void checkGyrCal(void)
{
	float duration = LPMS_GYR_CALIBRATION_DURATION_30S;
	float stDuration = duration / 10.0f;

	if (isGyrCalibrationEnabled) {
		gyrCalibrationDuration += lpmsMeasurementPeriod;
		++cumulatedGyrCounter;

		if (gyrCalibrationDuration < stDuration) {
			++startGyrCounter;
		}

		if (gyrCalibrationDuration > (duration - stDuration)) {
			++endGyrCounter;
		}
		
		for (uint8_t i = 0; i < 3; i++) {
			cumulatedGyrData[i] = cumulatedGyrData[i] + gyrRawData.data[i];

			if (gyrCalibrationDuration < stDuration) {
				startGyrData[i] = startGyrData[i] + gyrRawData.data[i];
			}
			
			if (maxGyr.data[i] < fabs(gyrRawData.data[i])) {
				maxGyr.data[i] = fabs(gyrRawData.data[i]);
			}

			if (gyrCalibrationDuration > (duration - stDuration)) {
				endGyrData[i] = endGyrData[i] + gyrRawData.data[i];
			}			

			if (minGyr.data[i] > fabs(gyrRawData.data[i])) {
				minGyr.data[i] = fabs(gyrRawData.data[i]);
			}
		}
		
		if (gyrCalibrationDuration >= LPMS_GYR_CALIBRATION_DURATION_30S) {
			stopGyrCalibration();
		}
	}
}

void stopGyrCalibration(void)
{
	float o;
	
	for (uint8_t i = 0; i < 3; i++) {
		o = (float)cumulatedGyrData[i] / (float)cumulatedGyrCounter;
		calibrationData.gyrOffset.data[i] = o;
		gReg.data[LPMS_GYR_BIAS_X + i] = conFtoI(o);

		gDrift.data[i] = ((float)((float)endGyrData[i] / (float)endGyrCounter) - (float)((float)startGyrData[i] / (float)startGyrCounter)) / LPMS_GYR_CALIBRATION_DURATION_30S;
		
		lpFilterParam.gyrThreshold.data[i] = (float) maxGyr.data[i] - o;
		gReg.data[LPMS_GYR_THRES_X + i] = conFtoI(lpFilterParam.gyrThreshold.data[i]);
		
		cumulatedGyrData[i] = 0;
	}
	
	isGyrCalibrationEnabled = 0;
	cumulatedGyrCounter = 0;
	gyrCalibrationDuration = 0;
	
	lpmsStatus &= ~LPMS_GYR_CALIBRATION_RUNNING;		
}

void startRefCalibration(void)
{
  	if (isRefCalibrationEnabled == 1) return;
  	
	isRefCalibrationEnabled = 1;

	maxGyr.data[0] = 0;
	maxGyr.data[1] = 0;
	maxGyr.data[2] = 0;	

	cumulatedRefCounter = 0;
	refCalibrationDuration = 0.0f;
	
	for (uint8_t i = 0; i < 3; i++) cumulatedRefData[i] = 0;

	lpmsStatus |= LPMS_REF_CALIBRATION_RUNNING;	
}

void checkRefCal(void)
{
	float bInc;
	LpVector3f tR;

	if (isRefCalibrationEnabled) {
		refCalibrationDuration += lpmsMeasurementPeriod;

		cumulatedRefCounter++;
		
		getReferenceYZ(b, a, &tR, &bInc);
		
		cumulatedRefData[0] = cumulatedRefData[0] + bInc;
		cumulatedRefData[1] = cumulatedRefData[1] + tR.data[1];
		cumulatedRefData[2] = cumulatedRefData[2] + tR.data[2];
		cumulatedRefData[2] = cumulatedRefData[2] + tR.data[2];
		
		if (refCalibrationDuration >= LPMS_REF_CALIBRATION_DURATION_01S) {
			stopRefCalibration();
		}
	}
}

void stopRefCalibration(void)
{
	float o;
	float n;

  	if (isRefCalibrationEnabled == 0) return;

	o = cumulatedRefData[0] / (float) cumulatedRefCounter;  
	lpFilterParam.magInclination = o;
	gReg.data[LPMS_MAG_FIELD_INC] = conFtoI(o);

	cumulatedRefData[0] = 0;
	lpFilterParam.magRef.data[0] = 0.0f;

	for (uint8_t i = 1; i < 3; i++) {
		o = cumulatedRefData[i] / (float) cumulatedRefCounter;
		lpFilterParam.magRef.data[i] = o;
		cumulatedRefData[i] = 0;
	}

	n = vect3x1Norm(lpFilterParam.magRef);
	scalarVectMult3x1(n, &lpFilterParam.magRef, &lpFilterParam.magRef);

	gReg.data[LPMS_MAG_REF_X + 0] = conFtoI(0.0f);
	gReg.data[LPMS_MAG_REF_X + 1] = conFtoI(lpFilterParam.magRef.data[1]);
	gReg.data[LPMS_MAG_REF_X + 2] = conFtoI(lpFilterParam.magRef.data[2]);

	lpFilterParam.accRef.data[0] = 0.0f;
	gReg.data[LPMS_ACC_REF_X + 0] = conFtoI(0.0f);
	lpFilterParam.accRef.data[1] = 0.0f;
	gReg.data[LPMS_ACC_REF_Y + 0] = conFtoI(0.0f);
	lpFilterParam.accRef.data[2] = -1.0f;
	gReg.data[LPMS_ACC_REF_Z + 0] = conFtoI(-1.0f);
	
	qOffset.data[0] = 1.0f;
	qOffset.data[1] = 0.0f;
	qOffset.data[2] = 0.0f;
	qOffset.data[3] = 0.0f;

	setRegVector4f(LPMS_OFFSET_QUAT_0, qOffset);

	isRefCalibrationEnabled = 0;
	
	lpmsStatus &= ~LPMS_REF_CALIBRATION_RUNNING;	
}

uint8_t getSensorDataAscii(uint8_t* data, uint16_t *l)
{
  	uint16_t o = 0;
    	uint8_t sl;
	float mT = measurementTime * lpmsMeasurementPeriod; // lpmsMeasurementPeriod;

	setUi32tAscii(&(data[o]), &sl, (uint32_t)(mT * 1000.0f)); o = o+sl;
	data[o] = ','; ++o;
	
	if ((gReg.data[LPMS_CONFIG] & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
		for (int i=0; i<3; i++) {
			setFloatAscii(&(data[o]), &sl, gRaw.data[i], FLOAT_FIXED_POINT_1000); o = o+sl;
			data[o] = ','; ++o;
		}
	}
	
	if ((gReg.data[LPMS_CONFIG] & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
		for (int i=0; i<3; i++) {
			setFloatAscii(&(data[o]), &sl, aRaw.data[i], FLOAT_FIXED_POINT_1000); o = o+sl;
			data[o] = ','; ++o;
		}
	}	

	if ((gReg.data[LPMS_CONFIG] & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
		for (int i=0; i<3; i++) {
			setFloatAscii(&(data[o]), &sl, bRaw.data[i], FLOAT_FIXED_POINT_100); o = o+sl;
			data[o] = ','; ++o;
		}
	}

	if ((gReg.data[LPMS_CONFIG] & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
		for (int i=0; i<3; i++) {
			setFloatAscii(&(data[o]), &sl, w.data[i], FLOAT_FIXED_POINT_1000); o = o+sl;
			data[o] = ','; ++o;
		}
	}
	
	if ((gReg.data[LPMS_CONFIG] & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
		for (int i=0; i<4; i++) {
			setFloatAscii(&(data[o]), &sl, qAfterOffset.data[i], FLOAT_FIXED_POINT_1000); o = o+sl;
			data[o] = ','; ++o;
		}
	}	
		
	if ((gReg.data[LPMS_CONFIG] & LPMS_EULER_OUTPUT_ENABLED) != 0)  {
		for (int i=0; i<3; i++) {
			setFloatAscii(&(data[o]), &sl, rAfterOffset.data[i], FLOAT_FIXED_POINT_1000); o = o+sl;
			data[o] = ','; ++o;
		}
	}	

	if ((gReg.data[LPMS_CONFIG] & LPMS_LINACC_OUTPUT_ENABLED) != 0) {
		for (int i=0; i<3; i++) {
			setFloatAscii(&(data[o]), &sl, linAcc.data[i], FLOAT_FIXED_POINT_1000); o = o+sl;
			data[o] = ','; ++o;
		}
	}

	if ((gReg.data[LPMS_CONFIG] & LPMS_PRESSURE_OUTPUT_ENABLED) != 0)  {
		setFloatAscii(&(data[o]), &sl, pressure, FLOAT_FIXED_POINT_100); o = o+sl;
		data[o] = ','; ++o;
	}

	if ((gReg.data[LPMS_CONFIG] & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0)  {
		setFloatAscii(&(data[o]), &sl, altitude, FLOAT_FIXED_POINT_10); o = o+sl;
		data[o] = ','; ++o;
	}	

	if ((gReg.data[LPMS_CONFIG] & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0)  {
		setFloatAscii(&(data[o]), &sl, temperature, FLOAT_FIXED_POINT_100); o = o+sl;
		data[o] = ','; ++o;
	}

#ifdef USE_HEAVEMOTION
	if ((gReg.data[LPMS_CONFIG] & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
		setFloatAscii(&(data[o]), &sl, heaveY, FLOAT_FIXED_POINT_1000); o = o+sl;
		data[o] = ','; ++o;
	}
#endif

	*l = o;

    return 1;
}

uint8_t getSensorData(uint8_t* data, uint16_t *l)
{
  	uint16_t o = 0;
	float mT = measurementTime * lpmsMeasurementPeriod; // lpmsMeasurementPeriod;

#ifdef LPMS_BLE
	if (connectedInterface == USB_CONNECTED) {
                setUi32t(&(data[o]), (uint32_t)(mT * 1000.0f));
                o = o+4;

                if ((gReg.data[LPMS_CONFIG] & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<4; i++) {
                                setFloat(&(data[i*2 + o]), qAfterOffset.data[i], FLOAT_FIXED_POINT_1000);
                        }
                        o = o+8;
                }

                if ((gReg.data[LPMS_CONFIG] & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
                        setFloat(&(data[o]), heaveY, FLOAT_FIXED_POINT_1000);
                        o = o+2;
                }
	} else {
		setI16t(&(data[o]), (int16_t) mT);
		o = o+2;
		
		setI16t(&(data[o]), (int16_t) (qAfterOffset.data[0] * (float) 0x7fff));
		o = o+2;
	
		setI16t(&(data[o]), (int16_t) (qAfterOffset.data[1] * (float) 0x7fff));
		o = o+2;
	
		setI16t(&(data[o]), (int16_t) (qAfterOffset.data[2] * (float) 0x7fff));
		o = o+2;
	
		setI16t(&(data[o]), (int16_t) (qAfterOffset.data[3] * (float) 0x7fff));
		o = o+2;
	
		setI16t(&(data[o]), (int16_t) (heaveY * (float) 0x0fff));
		o = o+2;
	}
#else
        if ((gReg.data[LPMS_CONFIG] & LPMS_LPBUS_DATA_MODE_16BIT_ENABLED) != 0) {
                setFloat(&(data[o]), mT, FLOAT_FULL_PRECISION);
		  		o = o+4;
                
                if ((gReg.data[LPMS_CONFIG] & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<3; i++) {
#ifdef ENABLE_INSOLE
								setFloat(&(data[i*2+ o]), forceSensorOutput[i], FLOAT_FIXED_POINT_1000);
#else
                                setFloat(&(data[i*2 + o]), gRaw.data[i], FLOAT_FIXED_POINT_1000);
#endif
                        }
                        o = o+6;
                }
                
                if ((gReg.data[LPMS_CONFIG] & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<3; i++) {
                                setFloat(&(data[i*2 + o]), aRaw.data[i], FLOAT_FIXED_POINT_1000);
                        }
                        o = o+6;
                }	
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<3; i++) {
                                setFloat(&(data[i*2 + o]), bRaw.data[i], FLOAT_FIXED_POINT_100);
                        }
                        o = o+6;
                }
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<3; i++) {
                                setFloat(&(data[i*2 + o]), w.data[i], FLOAT_FIXED_POINT_1000);
                        }
                        o = o+6;
                }
                
                if ((gReg.data[LPMS_CONFIG] & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<4; i++) {
                                setFloat(&(data[i*2 + o]), qAfterOffset.data[i], FLOAT_FIXED_POINT_1000);
                        }
                        o = o+8;
                }	
                        
                if ((gReg.data[LPMS_CONFIG] & LPMS_EULER_OUTPUT_ENABLED) != 0)  {
                        for (int i=0; i<3; i++) {
                                setFloat(&(data[i*2 + o]), rAfterOffset.data[i], FLOAT_FIXED_POINT_1000);
                        }
                        o = o+6;
                }	
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_LINACC_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<3; i++) {
                                setFloat(&(data[i*2+ o]), linAcc.data[i], FLOAT_FIXED_POINT_1000);
                        }
                        o = o+6;
                }
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_PRESSURE_OUTPUT_ENABLED) != 0)  {
#ifdef ENABLE_INSOLE
                        setFloat(&(data[0 + o]), forceSensorOutput[3], FLOAT_FIXED_POINT_10);
#else
                        setFloat(&(data[0 + o]), pressure, FLOAT_FIXED_POINT_10);
#endif

                        o = o+2;
                }
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0)  {
                        setFloat(&(data[0 + o]), altitude, FLOAT_FIXED_POINT_10);
                        o = o+2;
                }	
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0)  {
                        setFloat(&(data[0 + o]), temperature, FLOAT_FIXED_POINT_100);
                        o = o+2;
                }
        
#ifdef USE_HEAVEMOTION
                if ((gReg.data[LPMS_CONFIG] & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
                        setFloat(&(data[o]), heaveY, FLOAT_FIXED_POINT_1000);
                        o = o+2;
                }
#endif
        } else {
                setFloat(&(data[o]), mT, FLOAT_FULL_PRECISION);
                o = o+4;
                
                if ((gReg.data[LPMS_CONFIG] & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<3; i++) {
#ifdef ENABLE_INSOLE
								setFloat(&(data[i*4 + o]), forceSensorOutput[i], FLOAT_FULL_PRECISION);
#else
                                setFloat(&(data[i*4 + o]), gRaw.data[i], FLOAT_FULL_PRECISION);
#endif
                        }
                        o = o+12;
                }
                
                if ((gReg.data[LPMS_CONFIG] & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<3; i++) {
                                setFloat(&(data[i*4 + o]), aRaw.data[i], FLOAT_FULL_PRECISION);
                        }
                        o = o+12;
                }	
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<3; i++) {
							setFloat(&(data[i*4 + o]), bRaw.data[i], FLOAT_FULL_PRECISION);
                        }
                        o = o+12;
                }
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<3; i++) {
                                setFloat(&(data[i*4 + o]), w.data[i], FLOAT_FULL_PRECISION);
                        }
                        o = o+12;
                }
                
                if ((gReg.data[LPMS_CONFIG] & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<4; i++) {
	                        setFloat(&(data[i*4 + o]), qAfterOffset.data[i], FLOAT_FULL_PRECISION);
                        }
                        o = o+16;
                }	
                        
                if ((gReg.data[LPMS_CONFIG] & LPMS_EULER_OUTPUT_ENABLED) != 0)  {
                        for (int i=0; i<3; i++) {
                                setFloat(&(data[i*4 + o]), rAfterOffset.data[i], FLOAT_FULL_PRECISION);
                        }
                        o = o+12;
                }	
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_LINACC_OUTPUT_ENABLED) != 0) {
                        for (int i=0; i<3; i++) {
                                setFloat(&(data[i*4 + o]), linAcc.data[i], FLOAT_FULL_PRECISION);
                        }
                        o = o+12;
                }
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_PRESSURE_OUTPUT_ENABLED) != 0)  {
#ifdef ENABLE_INSOLE
						setFloat(&(data[o]), forceSensorOutput[3], FLOAT_FULL_PRECISION);
#else
                        setFloat(&(data[o]), pressure, FLOAT_FULL_PRECISION);
#endif
                        o = o+4;
                }
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0)  {
                        setFloat(&(data[o]), altitude, FLOAT_FULL_PRECISION);
                        o = o+4;
                }	
        
                if ((gReg.data[LPMS_CONFIG] & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0)  {
                        setFloat(&(data[o]), temperature, FLOAT_FULL_PRECISION);
                        o = o+4;
                }
        
#ifdef USE_HEAVEMOTION
                if ((gReg.data[LPMS_CONFIG] & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
                        setFloat(&(data[o]), heaveY, FLOAT_FULL_PRECISION);
                        o = o+4;
                }
#endif
        }
#endif

	*l = o;
	
	return 1;
}