/***********************************************************************
** Functions to adjust sensor parameters
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_SENSOR_PARAMETERS_H
#define LPMS_SENSOR_PARAMETERS_H

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
#include "LpmsTimebase.h"

// Degree and radians indicators for data conversions 
#define USE_DEGREE 1
#define USE_RADIAN 0

// Retrieves curret IMU ID
uint16_t getImuID(void);

// Sets current IMU ID
void setImuID(uint16_t id);

// Updates current sensor parameters to sensor manager settings
void updateConfigRegToSensorManager(void);

// Sets current baudrate
uint8_t setBaudrate(uint8_t* data);

// Setes current streaming frequency
uint8_t setStreamFreq(uint8_t* data);

// Writes configuration registers to flash memory
uint8_t writeRegisters(void);

// Sets gyroscope range
uint8_t setGyrRange(uint8_t* data);

// Sets accelerometer range
uint8_t setAccRange(uint8_t* data);

// Sets magnetometer range
uint8_t setMagRange(uint8_t* data);

// Sets gyroscope output range
uint8_t setGyrOutputRate(uint8_t* data);

// Sets magnetometer output range
uint8_t setMagOutputRate(uint8_t* data);

// Sets accelerometer output range
uint8_t setAccOutputRate(uint8_t* data);

// Sets orientation offset
uint8_t setOrientationOffset(uint8_t* data);

// Resets the orientation offset
uint8_t resetOrientationOffset(uint8_t* data);

// Sets gyroscope bias
uint8_t setGyrBias(uint8_t* data);

// Sets accelerometer bias
uint8_t setAccBias(uint8_t* data);

// Sets magnetometer bias
uint8_t setMagBias(uint8_t* data);

// Retrieves current pitch data
uint8_t getPitchData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad);

// Retrieves current roll data
uint8_t getRollData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad);

// Retrieves current yaw data
uint8_t getYawData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad);

// Retrieves current sensor status
uint8_t getStatus(uint8_t* data);

// Retrives contents fof confirguration register
uint32_t getConfigReg(void);

// Enables/disables gyroscope threshold 
uint8_t setEnableGyrThresh(uint8_t* data); 

// Enables/disabels magnetometer auto-calibration
uint8_t setEnableMagAutoCal(uint8_t* data); 

// Enables/disables gyroscope auto-calibration
uint8_t setEnableGyrAutoCal(uint8_t* data);

// Sets accelerometer covariance
uint8_t setAccCovar(uint8_t* data);

// Sets accelerometer compensation gain
uint8_t setAccCompGain(uint8_t* data); 

// Sets magnetometer covariance
uint8_t setMagCovar(uint8_t* data);

// Sets magnetometer compensation gain
uint8_t setMagCompGain(uint8_t* data); 

// Sets filter preset
uint8_t setFilterPreset(uint8_t* data); 

// Sets filter mode
uint8_t setFilterMode(uint8_t* data);

// Sets CAN baudrate
uint8_t setCanBaudrate(uint8_t* data);

// Enables/disabels self test
uint8_t setSelfTest(uint8_t* data);

// Retrievs part of magnetic fieldmap
uint8_t getFieldMap(uint8_t* inData, uint8_t* data, uint16_t *l);

// Retrieves hard iron offset
uint8_t getHardIronOffsetData(uint8_t* data, uint16_t *l);

// Retrieves soft iron matrix
uint8_t getSoftIronMatrixData(uint8_t* data, uint16_t *l);

// Retrieves X Euler angle data
uint8_t getWXData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad);

// Retrieves Y Euler angle data
uint8_t getWYData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad);

// Retrieves Z Euler angle data
uint8_t getWZData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad);

// Sets hard iron offset
uint8_t setHardIronOffsetData(uint8_t* data);

// Sets soft iron matrix
uint8_t setSoftIronMatrixData(uint8_t* data);

// Sets magnetic field estimate
uint8_t setFieldEstimateData(uint8_t* data);

// Retrieves magnetic field estimate
uint8_t getFieldEstimateData(uint8_t* data, uint16_t *l);

// Retrieves accelerometer misalignment matrix
uint8_t getAccAlignMatrix(uint8_t* data, uint16_t *l);

// Sets accelerometer misalignment matrix
uint8_t setAccAlignMatrix(uint8_t* data);

// Retrieves accelerometer offset
uint8_t getAccOffset(uint8_t* data, uint16_t *l);

// Sets accelerometer offset
uint8_t setAccOffset(uint8_t* data);

// Sets transmission data selection
uint8_t setTransmitData(uint8_t* data);

// Retrieves quaternion W data
uint8_t getQ0Data(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves quaternion X data
uint8_t getQ1Data(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves quaternion Y data
uint8_t getQ2Data(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves quaternion Z data
uint8_t getQ3Data(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves linear acceleration X data
uint8_t getLinAccXData(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves linear acceleration Y data
uint8_t getLinAccYData(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves linear acceleration Z data
uint8_t getLinAccZData(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves magnetometer X data
uint8_t getMagXData(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves magnetometer Y data
uint8_t getMagYData(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves magnetometer Z data
uint8_t getMagZData(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves current firmware version
uint8_t getFirmwareVersion(uint8_t* data, uint16_t *l);

// Sets gyroscope misalignment matrix
uint8_t setGyrAlignMatrix(uint8_t* data);

// Sets gyroscvope misalignment bias
uint8_t setGyrAlignBias(uint8_t* data);

// Sets temperature calibration parameter A
uint8_t setGyrTempCalPrmA(uint8_t* data);

// Sets temperature calibration parameter B
uint8_t setGyrTempCalPrmB(uint8_t* data);

// Sets temperature calibration base V
uint8_t setGyrTempCalBaseV(uint8_t* data);

// Sets temperature calibration base T
uint8_t setGyrTempCalBaseT(uint8_t* data);

// Retrieves gyroscope misalignment matrix
uint8_t getGyrAlignMatrix(uint8_t* data, uint16_t *l);

// Retrieves gyroscope misalignment bias
uint8_t getGyrAlignBias(uint8_t* data, uint16_t *l);

// Retrieves acceleromter X data
uint8_t getAccXData(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves acceleromter Y data
uint8_t getAccYData(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves acceleromter Z data
uint8_t getAccZData(uint8_t* data, uint16_t *l, uint8_t prec);

// Retrieves current low-pass filter settings
uint8_t getRawDataLp(uint8_t* data, uint16_t *l);

// Sets current low-pass filter parameter
uint8_t setRawDataLp(uint8_t* data);

// Updates low-pass filter parameter register setting
void updateRawDataLp(void);

// Updates filter mode register settings
void updateFilterMode(void);

// Sets CAN bus data mapping
uint8_t setCanMapping(uint8_t* data);

// Updates CAN bus mapping
void updateCanMapping(void);

// Retrives CAN bus data mapping
uint8_t getCanMapping(uint8_t* data, uint16_t *l);

// Updates CAN heartbeat register setting
void updateCanHeartbeat(void);

// Sets CAN heartbeat parameter
uint8_t setCanHeartbeat(uint8_t* data);

// Retrievs CAN heartbeat parameter
uint8_t getCanHeartbeat(uint8_t* data, uint16_t *l);

// Sets linear acceleration compensation mode
uint8_t setLinAccCompMode(uint8_t* data);

// Updates linear acceleration register settings
void updateLinAccCompMode(void);

// Retrives linear acceleration compensation mode
uint8_t getLinAccCompMode(uint8_t* data, uint16_t *l);

// Sets rotation acceleration compensation mode
uint8_t setCentriCompMode(uint8_t* data);

// Updates rotation acceleration compensation mode
void updateCentriCompMode(void);

// Retrieves rotation acceleration compensation mode 
uint8_t getCentriCompMode(uint8_t* data, uint16_t *l);

// Retrieves CAN bus configuration parameters
uint8_t getCanConfiguration(uint8_t* data, uint16_t *l);

// Sets CAN channel mode parameters
uint8_t setCanChannelMode(uint8_t* data);

// Sets CAN point mode parameter
uint8_t setCanPointMode(uint8_t* data);

// Sets CAN start ID parameter
uint8_t setCanStartId(uint8_t* data);

// Retrives heave data
uint8_t getHeaveData(uint8_t* data, uint16_t *l, uint8_t prec);

uint8_t getGyroXData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad);
uint8_t getGyroYData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad);
uint8_t getGyroZData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad);
uint8_t setMagAlignMatrix(uint8_t* data);
uint8_t setMagAlignBias(uint8_t* data);
uint8_t setMagReference(uint8_t* data);
uint8_t getMagAlignMatrix(uint8_t* data, uint16_t *l);
uint8_t getMagAlignBias(uint8_t* data, uint16_t *l);
uint8_t getMagReference(uint8_t* data, uint16_t *l);
void updateMagAlignMatrix(void);
void updateMagAlignBias(void);
void updateMagReference(void);
void setDefaultMagReference(void);
void setDefaultAlignBias(void);
void setDefaultAlignMatrix(void);

#endif