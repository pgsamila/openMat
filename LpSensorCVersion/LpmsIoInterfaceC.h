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

#ifndef LPMS_IO_INTERFACE_C
#define LPMS_IO_INTERFACE_C

#include <stdint.h>

#include "../LpSensor/ImuData.h"
#include "../LpSensor/LpmsRegisterDefinitions.h"
#include "../LpSensor/LpMatrix.h"

#include "CalibrationDataC.h"
#include "MicroMeasureC.h"

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

#define FIRMWARE_PACKET_LENGTH 256
#define FIRMWARE_PACKET_LENGTH_LPMS_BLE 128

#define IDLE_STATE -1
#define ACK_MAX_TIME 3000000
#define MAX_UPLOAD_TIME 20000000
#define MAX_COMMAND_RESEND 3

typedef union _float2uint {
	float fp;
	uint32_t up;
} float2uint;

void lpmsInitIoInterface(CalibrationData *configData);

int lpmsConnect(char* deviceId);
void lpmsClose(void);
long long lpmsGetConnectWait(void);

void lpmsStartStreaming(void);
void lpmsStopStreaming(void);
int lpmsDeviceStarted(void);

int lpmsPollData(void);
int lpmsCheckState(void);
int lpmsGetCurrentState(void);
int lpmsParseFunction(void);
int lpmsParseSensorData(void);
void lpmsZeroImuData(ImuData* id);
int lpmsSendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
int lpmsParseModbusByte(unsigned char b);
void lpmsReceiveReset(void);
int lpmsIsAck(void);
int lpmsIsNack(void);

CalibrationData *lpmsGetConfigData(void);

int lpmsIsWaitForData(void);
int lpmsIsWaitForAck(void);
	
int lpmsIsCalibrating(void);
int lpmsIsError(void);	
	
int lpmsStartUploadFirmware(char* fn);
int lpmsStartUploadIap(char* fn);
int lpmsGetUploadProgress(int *p);
int lpmsHandleFirmwareFrame(void);
int lpmsHandleIAPFrame(void);
int lpmsCheckUploadTimeout(void);

int lpmsSetCommandMode(void);
int lpmsSetStreamMode(void);
int lpmsSetSleepMode(void);
int lpmsRestoreFactoryValues(void);
int lpmsSetSelfTest(long v);
int lpmsSelectData(long p);
int lpmsSetImuId(long v);
int lpmsSetBaudrate(long v);
int lpmsSetStreamFrequency(long v);
int lpmsStartGyrCalibration(void);
int lpmsSetGyrRange(long v);
int lpmsSetMagRange(long v);
int lpmsSetAccRange(long v);
int lpmsSetAccBias(LpVector3f v);
int lpmsSetMagRef(float x, float y, float z);
int lpmsSetFilterMode(long v);
int lpmsSetFilterPreset(long v);
int lpmsResetOrientation(void);
int lpmsEnableGyrThres(long v);
int lpmsWriteRegisters(void);
int lpmsSetCanStreamFormat(long v);
int lpmsSetCanBaudrate(long v);
int lpmsParseFieldMapData(void);
int lpmsGetHardIronOffset(void);
int lpmsGetSoftIronMatrix(void);
int lpmsEnableGyrAutocalibration(long v);
int lpmsSetHardIronOffset(LpVector3f v);
int lpmsSetSoftIronMatrix(LpMatrix3x3f m);
int lpmsSetFieldEstimate(float v);
int lpmsSetAccAlignment(LpMatrix3x3f m);
int lpmsSetGyrAlignment(LpMatrix3x3f m);
int lpmsSetGyrAlignmentBias(LpVector3f v);
int lpmsSetRawDataLpFilter(int v);
int lpmsSetCanMapping(int *v);
int lpmsSetCanHeartbeat(int v);
int lpmsSetLinAccCompMode(int v);
int lpmsSetCentriCompMode(int v);
int lpmsSetCanChannelMode(int v);
int lpmsSetCanPointMode(int v);
int lpmsSetCanStartId(int v);
int lpmsSetLpBusDataMode(int v);
int lpmsSetMagAlignmentMatrix(LpMatrix3x3f m);
int lpmsSetMagAlignmentBias(LpVector3f v);
int lpmsSetMagReference(LpVector3f v);
int lpmsSetOrientationOffset(void);
int lpmsResetOrientationOffset(void);
int lpmsSetTimestamp(float v);

int lpmsGetMode(void);
int lpmsGetHardIronOffset(void);
int lpmsGetSoftIronMatrix(void);
int lpmsGetFieldEstimate(void);
int lpmsGetAccAlignment(void);
int lpmsGetGyrAlignment(void);
int lpmsGetGyrAlignmentBias(void);
long lpmsGetConfigReg(void);
int lpmsGetRawDataLpFilter(void);
int lpmsGetCanMapping(void);
int lpmsGetCanHeartbeat(void);
int lpmsGetLinAccCompMode(void);
int lpmsGetCentriCompMode(void);
int lpmsGetCanConfiguration(void);
int lpmsGetLatestImuData(ImuData *id);
int lpmsGetFirmwareVersion(void);
int lpmsGetDeviceId(void);
int lpmsGetDeviceReleaseDate(void);
int lpmsGetConfig(void);
int lpmsGetImuId(void);
int lpmsGetStatus(void);
int lpmsGetBaudrate(void);
int lpmsGetStreamFreq(void);
int lpmsGetGyrRange(void);
int lpmsGetAccRange(void);
int lpmsGetMagRange(void);
int lpmsGetMagRef(void);
int lpmsGetSensorData(void);
int lpmsGetFilterMode(void);
int lpmsGetFilterPreset(void);
int lpmsGetMagAlignmentMatrix(void);
int lpmsGetMagAlignmentBias(void);
int lpmsGetMagReference(void);

uint32_t conFtoI(float f);
float conItoF(uint32_t v);

int modbusSetNone(unsigned command);

int modbusGet(unsigned command);
int modbusGetMultiUint32(unsigned command, uint32_t *v, int n); 

int modbusSetInt32(unsigned command, long v);
int modbusSetInt32Array(unsigned command, long *v, int length);
int modbusSetVector3Int32(unsigned command, long x, long y, long z);

int modbusSetFloat(unsigned command, float v);
int modbusSetVector3Float(unsigned command, float x, float y, float z);
int modbusSetMatrix3x3f(unsigned command, LpMatrix3x3f m);
int modbusSetVector3f(unsigned command, LpVector3f v);

int fromBufferInt32(unsigned char* data, int start, long *v);
int fromBufferVector3Int32(unsigned char* data, unsigned start, long *x, long *y, long *z);

int fromBufferFloat(unsigned char* data, unsigned start, float *v);
int fromBufferVector3Float(unsigned char* data, unsigned start, float *x, float *y, float *z);
int fromBufferVector4Float(unsigned char* data, unsigned start, float *q0, float *q1, float *q2, float *q3);

int fromBufferInt32Array(unsigned char* data, long *v, int length);
int fromBufferInt16(unsigned char* data, unsigned start, short *v);
int fromBuffer(unsigned char* data, unsigned start, long *x, long *y, long *z);

#endif
