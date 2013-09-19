/***********************************************************************
** Manages communication with sensors and data processing
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_SENSOR_MANAGER_H
#define LPMS_SENSOR_MANAGER_H

#include "stm32f2xx.h"     
#include "LpMatrix.h"
#include "LpmsConfig.h"
#include "LpmsL3gd20.h"
#include "LpmsBmp180.h"
#include "LpmsLsm303dlhc.h"
#include "LpFilterCVersion.h"
#include "LpmsTimebase.h"
#include "LpMagnetometerCalibration.h"
#include "LpmsTest.h"
#include "LpmsAssignments.h"
#include "LpmsSensorParameters.h"
#include "LpmsStartup.h"
#include "LpmsUsart.h"
#include "LpmsRn42.h"
#include "LpmsFactorySetting.h"
#include "HeaveMotion.h"

#define LPMS_MEASUREMENT_PERIOD 20

// Number of iterations for gyrsocope calibration
#define GYRO_ONLINE_CAL_ITER 			128

// Amplitude threshold for gyrsocope auto-calibration
#define GYR_CAL_THRES 				60

// Duration of gyroscope auto-calibration sequence
#define GYR_CAL_TIMEOUT 			5000

// Possible durations of manual gyroscope bias clibration
#define LPMS_GYR_CALIBRATION_DURATION_5S 	(5.0f)
#define LPMS_GYR_CALIBRATION_DURATION_10S	(10.0f)
#define LPMS_GYR_CALIBRATION_DURATION_15S 	(15.0f)
#define LPMS_GYR_CALIBRATION_DURATION_30S 	(30.0f)
#define LPMS_MAG_CALIBRATION_DURATION_10S 	(10.0f)
#define LPMS_MAG_CALIBRATION_DURATION_15S	(15.0f)
#define LPMS_MAG_CALIBRATION_DURATION_20S	(20.0f)
#define LPMS_MAG_CALIBRATION_DURATION_30S	(30.0f)
#define LPMS_REF_CALIBRATION_DURATION_5S	(5.0f)
#define LPMS_REF_CALIBRATION_DURATION_1S	(1.0f)

// Sensor calibration parameters
typedef struct _LpmsCalibrationData {
	LpVector3f gyrOffset;
	LpVector3f gyrGain;     

	uint32_t gyrRange;
	uint32_t gyrOutputRate;

	LpVector3f accGain;
	LpVector3f accOffset;
	uint32_t accRange;
	uint32_t accOutputRate;

	LpVector3f magGain;
	LpVector3f magOffset;
	LpMatrix3x3f magSoftIronMatrix;
	
	uint32_t magRange;
	uint32_t magOutputRate;
	
	LpMatrix3x3f gyrAlignment;
	LpMatrix3x3f accAlignment;
	LpMatrix3x3f magAlignment;

	LpVector3f gyrAlignOffset;
	LpVector3f gyrTempCalPrmA;
	LpVector3f gyrTempCalPrmB;
	LpVector3f gyrTempCalBaseV;
	float gyrTempCalBaseT;

	float lpAlpha;

	uint32_t canMapping[8];
	float canHeartbeatTiming;
} LpmsCalibrationData;

// LpFilter algorithm parameters
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

// Initializes the sensor manager
void initSensorManager(void);

// Reads new data from all sensors
void updateSensorData(void);

// Processes the current sensor data with the LpFilter for orientation calculation
void processSensorData(void);

// Retrieves the current sensor data
void getCurrentData(LpVector3i *cG, LpVector3i *cA, LpVector3i *cB, LpVector4f *cQ);

// Retrieves the current filter parameters
void getFilterParam(LpFilterParameters *lpFP);

// Sets the current filter parameters
void setFilterParam(LpFilterParameters lpFP);

// Retrieves the current sensor parameters
void getSensorParam(LpmsCalibrationData *lCD);

// Sets the current sensor parameters
void setSensorParam(LpmsCalibrationData lCD);

// Calculates raw accelerometer data running average
void accAverage(void);

// Calculates linear acceleration
void calcLinearAcceleration(void);

// Waits until sending data has been compleyted
void waitForSendCompleted(void);

// Does oone gyroscope calibration step
void checkGyrCal(void);

// Performs one magnetomteer calibration step
void checkMagCal(void);

// Starts calibration of magnetometer reference
void startRefCalibration(void);

// Performs next reference calibration step 
void checkRefCal(void);

// Stops reference calibration
void stopRefCalibration(void);

// Starts gyroscope calibration
void startGyrCalibration(void);

// Stops gyroscope calibration
void stopGyrCalibration(void);

// Starts magnetometer calibration
void startMagCalibration(void);

// Stops magnetometer calibration
void stopMagCalibration(void);

// Retrieves current sensor data
uint8_t getSensorData(uint8_t* data, uint16_t *l);

// Initializes watchdog timer
void initWatchdog(void);

// Converts raw gyroscope data to global angular velocities
void gyroToInertial(void);

// Applies low pass filter to raw data values
void applyLowPass(void);

// Calculates altitude based on pressure sensor values
void calcAltitude(void);

// Resets current timestamp
void resetTimestamp(void);

// Retrievs current transmission mode
uint8_t getCurrentMode(void);

// Sets current mode to command mode
void setCommandMode(void);

// Sets current mode to stream mode
void setStreamMode(void);

// Sets current mode to sleep mode
void setSleepMode(void);

// Sets data sending flag
void setDataSendingFlag(void);

int isDataSendingSet(void);

#endif