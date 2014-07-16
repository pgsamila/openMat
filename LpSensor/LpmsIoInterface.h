/***********************************************************************
** Copyright (C) 2013 LP-RESEARCH Inc.
** All rights reserved.
** Contact: info@lp-research.com
***********************************************************************/

#ifndef LPMS_IO_INTERFACE
#define LPMS_IO_INTERFACE

#include "ImuData.h"
#include "CalibrationData.h"
#include "LpmsRegisterDefinitions.h"
#include "LpMatrix.h"
#include "MicroMeasure.h"

#ifdef _WIN32
	#include "windows.h"
#endif

#include <iostream>
#include <string>
#include <queue>
#include <fstream>

#include <boost/cstdint.hpp>

// Logging to console or file
#define LOG_TO_CONSOLE
#ifdef LOG_TO_CONSOLE
	#define INIT_LOGGING
	#define LOGV(...) printf(__VA_ARGS__);
#else
	#define INIT_LOGGING 	static FILE *logFileHandle;
	#define LOGV(...) 		logFileHandle = fopen("LpmsLog.txt", "a"); \
							fprintf(logFileHandle, __VA_ARGS__); \
							fclose(logFileHandle);
#endif

// LP-BUS byte definitions
#define PACKET_ADDRESS0 0
#define PACKET_ADDRESS1 1
#define PACKET_FUNCTION0 2
#define PACKET_FUNCTION1 3
#define PACKET_RAW_DATA 4
#define PACKET_LRC_CHECK0 5
#define PACKET_LRC_CHECK1 6
#define PACKET_END 7
#define PACKET_LENGTH0 8
#define PACKET_LENGTH1 9
#define PACKET_SKIP_ZERO 10

// Firmware packet length
#define FIRMWARE_PACKET_LENGTH 256
#define FIRMWARE_PACKET_LENGTH_LPMS_BLE 128

// State machine definitions
#define IDLE_STATE -1
#define ACK_MAX_TIME 3000000
#define MAX_UPLOAD_TIME 20000000
#define MAX_COMMAND_RESEND 3

// Float to unsigned integer conversion structure
typedef union _float2uint {
	float fp;
	boost::uint32_t up;
} float2uint;

class BleBlock {
public:
	int bleConnectionHandle;
	int bleMeasurementHandle;
	unsigned char txData[32];
};

// Class for low-level communication with LPMS devices
class LpmsIoInterface {
public:
	// Constructor
	LpmsIoInterface(CalibrationData *configData);
	
	// Virtual functions to be overwritten by hardware dependent modules -->
	
	virtual ~LpmsIoInterface() { };	
	
	// Connects to device
	virtual bool connect(std::string deviceId) ;
	
	// Checks if device is started
	virtual bool deviceStarted(void);
	
	// Retrieves current data
	virtual void loadData(ImuData *data);
	
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
	
	// Retrieves gyroscope calibration cycles 
	virtual int getGyroCalCycles(void);

	// Checks current state machine state
	bool checkState(void);
	
	// Retrives current state machine state
	int getCurrentState(void);
	
	// Retrieves current configuraton parameters
	CalibrationData *getConfigData(void);
	
	// Checks if state machine is currently waiting for data
	bool isWaitForData(void);
	
	// Checks if state machine is currently waiting for acknowledge
	bool isWaitForAck(void);
	
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
	void zeroImuData(ImuData* id);
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
	bool isNewData(void);
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
	bool getLatestImuData(ImuData *id);
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
	
protected:
	virtual bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
	virtual bool parseModbusByte(void);
	virtual bool handleFirmwareFrame(void);
	virtual bool handleIAPFrame(void);	

	bool isAck(void);
	bool isNack(void);
	void receiveReset(void);
	bool parseFunction(void);
	boost::uint32_t conFtoI(float f);
	float conItoF(boost::uint32_t v);
	bool modbusSetNone(unsigned command);
	bool modbusGet(unsigned command);	
	bool modbusGetMultiUint32(unsigned command, boost::uint32_t *v, int n); 
	bool modbusSetInt32(unsigned command, long v);
	bool modbusSetInt32Array(unsigned command, long *v, int length);
	bool modbusSetVector3Int32(unsigned command, long x, long y, long z);
	bool modbusSetFloat(unsigned command, float v);
	bool modbusSetVector3Float(unsigned command, float x, float y, float z);
	bool fromBuffer(std::vector<unsigned char> data, long *v);
	bool fromBuffer(std::vector<unsigned char> data, long *x, long *y, long *z);
	bool fromBuffer(std::vector<unsigned char> data, float *v);
	bool fromBuffer(std::vector<unsigned char> data, float *x, float *y, float *z);
	bool fromBuffer(std::vector<unsigned char> data, long *v, int length);
	bool fromBuffer(std::vector<unsigned char> data, unsigned start, float *x, float *y, float *z);
	bool fromBuffer(std::vector<unsigned char> data, unsigned start, float *q0, float *q1, float *q2, float *q3);
	bool fromBuffer(std::vector<unsigned char> data, unsigned start, float *v);
	bool fromBuffer(unsigned char *data, float *v);
	bool fromBufferBigEndian(unsigned char *data, float *v);
	bool fromBufferInt16(unsigned char *data, int *v);
	bool fromBufferInt16(std::vector<unsigned char> data, unsigned start, short *v);
	bool fromBuffer(std::vector<unsigned char> data, unsigned start, long *x, long *y, long *z);
	virtual bool parseSensorData(void);
	bool modbusSetMatrix3x3f(unsigned command, LpMatrix3x3f m);
	bool modbusSetVector3f(unsigned command, LpVector3f v);
	bool checkUploadTimeout(void);

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
	std::queue<ImuData> imuDataQueue;
	std::vector<unsigned char> oneTx;
	unsigned lrcIndex;
	unsigned lrcCheck;
	unsigned lrcReceived;
	int rxState;
	int rawDataIndex;	
	bool waitForAck;
	bool ackReceived;
	bool dataReceived;
	int currentState;	
	long ackTimeout;
	bool waitForData;
	long dataTimeout;
	int pCount;
	ImuData imuData;
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
	MicroMeasure ackTimer;
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