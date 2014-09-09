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
#include "AdConverter.h"

#define LPMS_MEASUREMENT_PERIOD 0.0025f

#define PRESSURE_T 400

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
#define LPMS_REF_CALIBRATION_DURATION_01S	(0.1f) 

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

// Applies low pass filter to raw data values
void applyLowPass(void);

// Calculates altitude based on pressure sensor values
void calcAltitude(void);

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

uint8_t getSensorDataAscii(uint8_t* data, uint16_t *l);

void checkTimestampReset(void);

#endif