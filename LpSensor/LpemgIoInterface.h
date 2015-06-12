/***********************************************************************
** Copyright (C) LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** Redistribution and use in source and binary forms, with 
** or without modification, are permitted provided that the 
** following conditions are met:
**
** Redistributions of source code must retain the above copyright 
** notice, this list of conditions and the following disclaimer.
** Redistributions in binary form must reproduce the above copyright 
** notice, this list of conditions and the following disclaimer in 
** the documentation and/or other materials provided with the 
** distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
** FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#ifndef LPEMG_IO_INTERFACE
#define LPEMG_IO_INTERFACE

#include "EmgData.h"
#include "CalibrationData.h"
#include "LpmsRegisterDefinitions.h"
#include "LpMatrix.h"
#include "MicroMeasure.h"
#include "LpIoBase.h"

#ifdef _WIN32
	#include "windows.h"
#endif

#include <iostream>
#include <string>
#include <queue>
#include <fstream>

#include <boost/cstdint.hpp>

// Firmware packet length
#define FIRMWARE_PACKET_LENGTH 256
#define FIRMWARE_PACKET_LENGTH_LPMS_BLE 128

// State machine definitions
#define IDLE_STATE -1
#define ACK_MAX_TIME 3000000
#define MAX_UPLOAD_TIME 20000000
#define MAX_COMMAND_RESEND 3

// Class for low-level communication with LPMS devices
class LpemgIoInterface : public LpIoBase {
public:
	// Constructor
	LpemgIoInterface(CalibrationData *configData);
	
	virtual ~LpemgIoInterface() { };	
	
	// Connects to device
	virtual bool connect(std::string deviceId) ;
	
	// Checks if device is started
	virtual bool deviceStarted(void);
	
	// Retrieves current data
	virtual void loadData(EmgData *data);
	
	// Polls data from device
	virtual bool pollData(void);
	
	// Closes data connection
	virtual void close(void);
	
	// Starts streaming data from device
	virtual void startStreaming(void);
	
	// Stops streaming data from device
	virtual void stopStreaming(void);
	
	// Retrieves connection delay
	virtual long long getConnectWait(void);
	
	// Retrieves sampling time
	virtual float getSamplingTime(void);

	// Checks current state machine state
	bool checkState(void);
	
	// Retrieves current configuraton parameters
	CalibrationData *getConfigData(void);
	
	// Checks if state machine is currently waiting for data
	// bool isWaitForData(void);
	
	// Checks if state machine is currently waiting for acknowledge
	// bool isWaitForAck(void);
	
	// Checks if currently calibrating
	bool isCalibrating(void);
	
	// Checks if error has occurred
	bool isError(void);	
	
	// Retrieves upload progress
	bool getUploadProgress(int *p);
	
	// Starts uploading firmware
	virtual bool startUploadFirmware(std::string fn);
	
	// Starts uploading IAP
	virtual bool startUploadIap(std::string fn);
	
	// Sets sensor to command mode
	bool setCommandMode(void);
	
	// Sets sensor to streaming mode
	bool setStreamMode(void);
	
	// Sets sensor to sleep mode
	bool setSleepMode(void);

	// Restores factory settings
	bool restoreFactoryValue(void);
	
	// Starts self test
	bool setSelfTest(long v);
	
	// Selects data for transmission
	bool selectData(long p);
	
	// Sets IMU ID
	bool setImuId(long v);
	
	// Sets Baudrate
	bool setBaudrate(long v);
	
	// Sets stream frequency
	bool setStreamFrequency(long v);
	
	// Starts gyroscope calibration
	bool startGyrCalibration(void);
	bool startAccCalibration(void);
	bool startMagCalibration(void);
	bool setGyrRange(long v);
	bool setMagRange(long v);
	bool setAccRange(long v);
	bool setGyrOutputRate(long v);
	bool setMagOutputRate(long v);
	bool setAccOutputRate(long v);
	bool setGyrBias(long x, long y, long z);
	bool setAccBias(LpVector3f v);
	bool setMagBias(long x, long y, long z);
	bool setAccRef(float x, float y, float z);
	bool setMagRef(float x, float y, float z);
	bool setGyrThres(float x, float y, float z);
	bool setAccCovar(float v);
	bool setAccCompGain(float v);
	bool setMagCovar(float v);
	bool setMagCompGain(float v);
	bool setProcessCovar(float v);
	bool resetFilterParam(void);
	bool setFilterMode(long v);
	bool setFilterPreset(long v);
	bool getFirmwareVersion(void);
	bool getDeviceId(void);
	bool getDeviceReleaseDate(void);
	bool getConfig(void);
	bool getImuId(void);
	bool getStatus(void);
	bool getBaudrate(void);
	bool getStreamFreq(void);
	bool getGyrRange(void);
	bool getAccRange(void);
	bool getMagRange(void);
	bool getGyrOutputRate(void);
	bool getAccOutputRate(void);
	bool getMagOutputRate(void);
	bool getGyrBias(void);
	bool getAccBias(void);
	bool getMagBias(void);
	bool getAccRef(void);
	bool getMagRef(void);
	bool getGyrThres(void);
	bool getAccCovar(void);
	bool getAccCompGain(void);
	bool getMagCovar(void);
	bool getMagCompGain(void);
	bool getProcessCovar(void);
	bool getSensorData(void);
	bool getFilterMode(void);
	bool getFilterPreset(void);	
	bool resetOrientation(void);
	bool enableGyrThres(long v);
	bool writeRegisters(void);
	bool enableMagAutocalibration(long v);
	int getMode(void);
	bool setCanStreamFormat(long v);
	bool setCanBaudrate(long v);
	float getLatestLatency(void);
	bool parseFieldMapData(void);
	bool getHardIronOffset(void);
	bool getSoftIronMatrix(void);
	bool setHardIronOffset(void);
	bool setSoftIronMatrix(void);
	bool enableGyrAutocalibration(long v);
	void zeroData(EmgData* d);
	bool setHardIronOffset(LpVector3f v);
	bool setSoftIronMatrix(LpMatrix3x3f m);
	bool setFieldEstimate(float v);
	bool getFieldEstimate(void);
	bool setAccAlignment(LpMatrix3x3f m);
	bool getAccAlignment(void);
	bool setGyrAlignment(LpMatrix3x3f m);
	bool setGyrAlignmentBias(LpVector3f v);
	bool setGyrTempCalPrmA(LpVector3f v);
	bool setGyrTempCalPrmB(LpVector3f v);
	bool setGyrTempCalBaseV(LpVector3f v);
	bool setGyrTempCalBaseT(float v);
	bool getGyrAlignment(void);
	bool getGyrAlignmentBias(void);
	bool getGyrTempCalPrmA(void);
	bool getGyrTempCalPrmB(void);
	bool getGyrTempCalBaseV(void);
	bool getGyrTempCalBaseT(void);
	long getConfigReg(void);
	bool getRawDataLpFilter(void);
	bool setRawDataLpFilter(int v);
	bool getCanMapping(void);
	bool setCanMapping(int *v);
	bool getCanHeartbeat(void);
	bool setCanHeartbeat(int v);
	bool resetSensorTimestamp(void);
	bool getLinAccCompMode(void);
	bool setLinAccCompMode(int v);
	bool getCentriCompMode(void);
	bool setCentriCompMode(int v);
	bool getCanConfiguration(void);
	bool setCanChannelMode(int v);
	bool setCanPointMode(int v);
	bool setCanStartId(int v);
	bool getLatestData(EmgData *d);
	void clearRxBuffer(void);
	void clearDataQueue(void);
	void setTxRxImuId(int id);
	bool setLpBusDataMode(int v);
	bool setMagAlignmentMatrix(LpMatrix3x3f m);
	bool setMagAlignmentBias(LpVector3f v);
	bool setMagReference(LpVector3f v);
	bool getMagAlignmentMatrix(void);
	bool getMagAlignmentBias(void);
	bool getMagReference(void);
	bool setOrientationOffset(int v);
	bool resetOrientationOffset(void);
	bool setTimestamp(float v);
	bool getUartBaudRate(void);
	bool setUartBaudRate(int v);
	bool setUartFormat(int v);
	bool armTimestampReset(int v);
	bool parseSensorData(void);	
	
protected:
	// Virtual functions to be overwritten by hardware dependent modules -->
	// virtual bool parseModbusByte(void);	

	// Firmware / IAP upload handling
	bool handleFirmwareFrame(void);
	bool handleIAPFrame(void);
	bool parseFunction(void);
	bool checkUploadTimeout(void);

protected:
	// Internal variables
	int fieldMapPitch;
	int fieldMapRoll;
	int fieldMapYaw;
	int currentFieldMapPitch;
	int currentFieldMapRoll;
	int currentFieldMapYaw;
	unsigned currentAddress;
	unsigned currentFunction;
	unsigned currentLength;
	std::queue<unsigned char> dataQueue;
	std::queue<EmgData> emgDataQueue;
	std::vector<unsigned char> oneTx;
	unsigned lrcIndex;
	unsigned lrcCheck;
	unsigned lrcReceived;
	int rxState;
	int rawDataIndex;	
	bool waitForAck;
	bool dataReceived;
	bool waitForData;
	long dataTimeout;
	int pCount;
	EmgData emgData;
	CalibrationData *configData;
	std::ifstream ifs;	
	long configReg;
	long lpmsStatus;
	long imuId;
	long long firmwarePages;
	int currentMode;
	float latestLatency;
	MicroMeasure latencyTimer;
	MicroMeasure uploadTimer;
	bool newDataFlag;
	float timestampOffset;
	float currentTimestamp;
	unsigned char cBuffer[512];
	int resendI;
	int cLength;
	bool isOpen;
	int firmwarePageSize;
};	

#endif