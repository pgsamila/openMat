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

#include "LpmsIoInterfaceC.h"

#ifdef _WIN32
	#include "windows.h"
#endif

#ifdef __VISUALDSPVERSION__
	#include <environment.h>
	#include <datatypes.h>
	#include <uart/uart.h>
	#include <buffermngt/bsbuf.h>
#endif

INIT_LOGGING

int fieldMapPitch;
int fieldMapRoll;
int fieldMapYaw;
int currentFieldMapPitch;
int currentFieldMapRoll;
int currentFieldMapYaw;
int rxState;
int rawDataIndex;	
int waitForAck;
int ackReceived;
int dataReceived;
int currentState;
int waitForData;
int pCount;
int currentMode;
int newDataFlag;
int resendIndex;
int cLength;
int isOpen;
int firmwarePageSize;
int isOpen;
int lpmsIoInterfaceReceivedNewData;

unsigned currentAddress;
unsigned currentFunction;
unsigned currentLength;
unsigned lrcIndex;
unsigned lrcCheck;
unsigned lrcReceived;

float latestLatency;
float timestampOffset;
float currentTimestamp;

unsigned char oneTx[256];
char portname[64];
char idNumber[64];
unsigned char cBuffer[512];

long ackTimeout;
long dataTimeout;
long configReg;
long lpmsStatus;
long imuId;
long long firmwarePages;

ImuData imuData;
CalibrationData *configData;

#ifdef _WIN32
	HANDLE rs232Handle;
	DCB rs232Config;
#endif

#ifdef __VISUALDSPVERSION__
	T_UART_HANDLE adBfUartHandle;
#endif

/* std::queue<unsigned char> dataQueue;
std::queue<ImuData> imuDataQueue; */

// std::ifstream ifs;	

/* MicroMeasure latencyTimer;
MicroMeasure uploadTimer;
MicroMeasure ackTimer; */



/***********************************************************************
** SENSOR-SPECIFIC FUNCTIONALITY
***********************************************************************/
int lpmsConnect(char* deviceId)
{
#ifdef _WIN32
    COMMTIMEOUTS comTimeOut;

	rs232Handle = CreateFile("\\\\.\\COM159", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

	if (GetCommState(rs232Handle, &rs232Config) == 0) {
		LOGV("[LpmsIoInterfaceC] Couldn't connect to COM port.\n");
		return 0;
    }

	rs232Config.BaudRate = 921600;
	rs232Config.StopBits = ONESTOPBIT;
	rs232Config.Parity = NOPARITY;     
	rs232Config.ByteSize = 8;  

	if (SetCommState(rs232Handle, &rs232Config) == 0) {
		LOGV("[LpmsIoInterfaceC] Couldn't set COM configuration.\n");	
		return 0;
	}
               
    comTimeOut.ReadIntervalTimeout = 20;
    comTimeOut.ReadTotalTimeoutMultiplier = 20;
    comTimeOut.ReadTotalTimeoutConstant = 20;
    comTimeOut.WriteTotalTimeoutMultiplier = 20;
    comTimeOut.WriteTotalTimeoutConstant = 20;
	
    SetCommTimeouts(rs232Handle, &comTimeOut);
#endif

#ifdef __VISUALDSPVERSION__
	T_ERROR_CODE errorCode;
	T_UART_CONFIG adBfUartConf;

	UARTsetup();
	
	UARTinitConfig(&adBfUartConf);
	
	adBfUartConf.ucUART = LPMS_UART_NR; 
	adBfUartConf.unBaudrate = 115200;
	adBfUartConf.ucDataBits = 8;
	adBfUartConf.tStopBits = UART_ONE_STOPBIT;
	adBfUartConf.tParity = UART_PARITY_NONE;
	adBfUartConf.tFlowCtrl = UART_FLOW_CTRL_NONE;
	adBfUartConf.unSystemClk = PWRgetSystemClock();
	adBfUartConf.unCoreClk = PWRgetCoreClock();
	adBfUartConf.tMode = UART_MODE_TX_DMA;
	adBfUartConf.unRxBufSize = 256;
	adBfUartConf.unTxBufSize = 256;
	adBfUartConf.fnRxCallback = NULL;
	adBfUartConf.pRxCallbackArg = 0;
	adBfUartConf.fnTxCallback = NULL;
	adBfUartConf.pTxCallbackArg = 0;
	adBfUartConf.fnSetRxDir = NULL;
	adBfUartConf.fnSetTxDir = NULL;
	
	adBfUartHandle = UARTopen(&adBfUartConf, &errorCode);
	
	if (errorCode != ERR_NONE) {
		LOGV("[LpmsIoInterfaceC] Couldn't connect to COM port.\n");
		return 0;
	}
	
	unsigned char aucBuf[20];
	aucBuf[0] = 8;
	UARTcontrol(adBfUartHandle, UART_SET_MIN_PACKET_SIZE_TO_SEND, aucBuf);
#endif

	strcpy(idNumber, deviceId);
	
	isOpen = 0;
	rawDataIndex = 0;
	
	rxState = PACKET_END;
	currentState = GET_CONFIG;
	
	waitForAck = 0;
	ackReceived = 0;
	waitForData = 0;
	dataReceived = 0;
	pCount = 0;
	ackTimeout = 0;
	dataTimeout = 0;	
	lpmsStatus = 0;
	configReg = 0;
	lpmsIoInterfaceReceivedNewData = 0;
	
	isOpen = 1;
	
	LOGV("[LpmsIoInterfaceC] LPMS connection is open.\n");
	
	return 1;
}

int lpmsWrite(unsigned char *txBuffer, unsigned bufferLength)
{
	unsigned long l;
	
#ifdef _WIN32
	if (WriteFile(rs232Handle, txBuffer, bufferLength, &l, NULL) == 0) {
		return 0;
    }
#endif

#ifdef __VISUALDSPVERSION__
	T_ERROR_CODE errorCode;
	l = UARTwrite(adBfUartHandle, txBuffer, bufferLength, &errorCode);
#endif
	
	return 1;
}

int lpmsRead(unsigned char *rxBuffer, unsigned long *bytesReceived) {

#ifdef _WIN32
	if (ReadFile(rs232Handle, rxBuffer, 64, bytesReceived, NULL) == 0) {
		return 0;
	}
#endif

#ifdef __VISUALDSPVERSION__
	T_ERROR_CODE errorCode;
	*bytesReceived = UARTread(adBfUartHandle, rxBuffer, 64, &errorCode);
#endif

	return 1;
}

void lpmsClose(void) {	
	if (isOpen == 0) return;

	isOpen = 0;
	
#ifdef _WIN32
	CloseHandle(rs232Handle);
#endif

#ifdef __VISUALDSPVERSION__
	UARTclose(adBfUartHandle);
#endif
	
	return;
}

int lpmsSendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{
	unsigned char txData[256];
	unsigned int txLrcCheck;
	int i;
	
	if (length > 255) return 0;

	txData[0] = 0x3a;
	txData[1] = address & 0xff;
	txData[2] = (address >> 8) & 0xff;
	txData[3] = function & 0xff;
	txData[4] = (function >> 8) & 0xff;
	txData[5] = length & 0xff;
	txData[6] = (length >> 8) & 0xff;
	
	for (i=0; i < length; ++i) {
		txData[7+i] = data[i];
	}
	
	txLrcCheck = address;
	txLrcCheck += function;
	txLrcCheck += length;
	
	for (i=0; i < length; i++) {
		txLrcCheck += data[i];
	}
	
	txData[7 + length] = txLrcCheck & 0xff;
	txData[8 + length] = (txLrcCheck >> 8) & 0xff;
	txData[9 + length] = 0x0d;
	txData[10 + length] = 0x0a;
	
	if (lpmsWrite(txData, length+11) == 1) {
		return 1;
	}
	
	return 0;
}

int lpmsParseModbusByte(unsigned char b)
{
	int i=0;

	switch (rxState) {
	case PACKET_END:
		if (b == 0x3a) {
			rxState = PACKET_ADDRESS0;
		}
	break;
		
	case PACKET_ADDRESS0:
		currentAddress = b;
		rxState = PACKET_ADDRESS1;
	break;

	case PACKET_ADDRESS1:
		currentAddress = currentAddress + ((unsigned) b * 256);
		rxState = PACKET_FUNCTION0;
	break;

	case PACKET_FUNCTION0:
		currentFunction = b;
		rxState = PACKET_FUNCTION1;				
	break;

	case PACKET_FUNCTION1:
		currentFunction = currentFunction + ((unsigned) b * 256);
		rxState = PACKET_LENGTH0;			
	break;

	case PACKET_LENGTH0:
		currentLength = b;
		rxState = PACKET_LENGTH1;
	break;
			
	case PACKET_LENGTH1:
		currentLength = currentLength + ((unsigned) b * 256);
		if (currentLength > 128) {
			rxState = PACKET_END;
			currentLength = 0;
		} else {
			rxState = PACKET_RAW_DATA;
			rawDataIndex = 0; 
		}
	break;
			
	case PACKET_RAW_DATA:
		if (rawDataIndex == currentLength) {
			lrcCheck = currentAddress + currentFunction + currentLength;
			for (i=0; i<currentLength; i++) lrcCheck += oneTx[i];
			lrcReceived = b;
			rxState = PACKET_LRC_CHECK1;			
		} else {	
			oneTx[rawDataIndex] = b;
			++rawDataIndex;
		}
	break;
		
	case PACKET_LRC_CHECK1:
		lrcReceived = lrcReceived + ((unsigned) b * 256);
		
		if (lrcReceived == lrcCheck) {
			lpmsParseFunction();
		} else {
			LOGV("[LpmsIoInterfaceC] LRC checksum fail.\n");
		}
		
		rxState = PACKET_END;
	break;
	
	default:
		rxState = PACKET_END;
	break;
	}
	
	return 1;
}

int lpmsPollData(void) 
{
	unsigned long bytesReceived;
	unsigned char rxBuffer[256];
	int packetOk = 0;
	int i;
	
	if (lpmsDeviceStarted() == 0) return 0;

	if (lpmsRead(rxBuffer, &bytesReceived) == 0) {
		isOpen = 0;
		return 0;
	}
	
	// LOGV("[LpmsIoInterfaceC] Received %d data bytes from LPMS.\n", bytesReceived);

	for (i=0; i < bytesReceived; i++) lpmsParseModbusByte(rxBuffer[i]);
	
	return 1;
}

int lpmsDeviceStarted(void)
{
	return isOpen;
}



/***********************************************************************
** COMMAND / DATA / ACK PARSING
***********************************************************************/
void lpmsInitIoInterface(CalibrationData *cd)
{
	configData = cd;
	firmwarePageSize = FIRMWARE_PACKET_LENGTH;
}

void lpmsZeroImuData(ImuData* id)
{
	int i, j;

	id->q[0] = 1.0f;
	id->q[1] = 0.0f;
	id->q[2] = 0.0f;
	id->q[3] = 0.0f;
	
	for (i=0; i<3; i++) id->r[i] = 0.0f;
	for (i=0; i<3; i++) id->a[i] = 0.0f;
	for (i=0; i<3; i++) id->g[i] = 0.0f;
	for (i=0; i<3; i++) id->b[i] = 0.0f;
	for (i=0; i<3; i++) id->aRaw[i] = 0.0f;
	for (i=0; i<3; i++) id->gRaw[i] = 0.0f;
	for (i=0; i<3; i++) id->bRaw[i] = 0.0f;
	for (i=0; i<3; i++) id->linAcc[i] = 0.0f;
	
	id->pressure = 0.0f;
	id->altitude = 0.0f;
	id->temperature = 0.0f;
	id->hm.yHeave = 0.0f;
	
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			if (i != j) {
				id->rotationM[i*3+j] = 0.0f;
				id->rotOffsetM[i*3+j] = 0.0f;
			} else {
				id->rotationM[i*3+j] = 1.0f;
				id->rotOffsetM[i*3+j] = 1.0f;
			}
		}
	}
	
	id->openMatId = 1;
	id->frameCount = 0;
	id->timeStamp = 0.0f;
	
	id->hm.yHeave = 0.0f;
	
	id->gm.zGait = 0.0f;
	id->gm.yGait = 0.0f;
	id->gm.zAmplitude = 0.0f;
	id->gm.yAmplitude = 0.0f;
	id->gm.frequency = 0.0f;
	id->gm.velocity = 0.0f;
	id->gm.symmetry = 0.0f;
	id->gm.zDirection = 0;
	id->gm.yDirection = 0;
}

int isAck(void) 
{
	if (currentFunction == REPLY_ACK) {
		return 1;
	}
	
	return 0;
}

int isNack(void) 
{
	if (currentFunction == REPLY_NACK) {
		return 1;
	}
	
	return 0;
}	

void lpmsReceiveReset(void) 
{
	currentState = IDLE_STATE;
	waitForData = 0;
	dataReceived = 0;
	waitForAck = 0;

	ackTimeout = 0;
	dataTimeout = 0;
	resendIndex = 0;
}

void lpmsPrintImuData(ImuData d)
{
	int i;

	LOGV("[LpmsIoInterfaceC] Gyr: ");
	for (i=0; i<3; ++i) LOGV("%f ", imuData.gRaw[i]);
	LOGV("\n");
	
	LOGV("[LpmsIoInterfaceC] Quat: ");
	for (i=0; i<4; ++i) LOGV("%f ", imuData.q[i]);
	LOGV("\n");
}

int parseSensorData(void)
{
	unsigned o;
	float r0, r1, r2;
	const float r2d = 57.2958f;
	short s;
	long l;
	int i;

	lpmsZeroImuData(&imuData);
	
	o = 0;
	if ((configReg & LPMS_LPBUS_DATA_MODE_16BIT_ENABLED) != 0) {
		fromBufferInt32(oneTx, 0, &l);
		currentTimestamp = (float) l / 10000.0f;
		o = o + 4;
		
		if ((configReg & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
			for (i=0; i<3; ++i) {
				fromBufferInt16(oneTx, o, &s);
				imuData.gRaw[i] = ((float)s) / 1000.0f * r2d;	
				o = o + 2;
			}		
		}
		
		if ((configReg & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
			for (i=0; i<3; ++i) {
				fromBufferInt16(oneTx, o, &s);
				imuData.aRaw[i] = ((float)s) / 1000.0f;	
				o = o + 2;
			}			
		}	

		if ((configReg & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
			for (i=0; i<3; ++i) {
				fromBufferInt16(oneTx, o, &s);
				imuData.bRaw[i] = ((float)s) / 100.0f;	
				o = o + 2;
			}			
		}
		
		if ((configReg & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
			for (i=0; i<3; ++i) {
				fromBufferInt16(oneTx, o, &s);
				imuData.w[i] = ((float)s) / 1000.0f * r2d;	
				o = o + 2;
			}				
		}
		
		if ((configReg & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
			for (i=0; i<4; ++i) {
				fromBufferInt16(oneTx, o, &s);
				imuData.q[i] = ((float)s) / 1000.0f;	
				o = o + 2;
			}		
		}	
			
		if ((configReg & LPMS_EULER_OUTPUT_ENABLED) != 0)  {
			for (i=0; i<3; ++i) {
				fromBufferInt16(oneTx, o, &s);
				imuData.r[i] = ((float)s) / 1000.0f * r2d;	
				o = o + 2;
			}
		}	

		if ((configReg & LPMS_LINACC_OUTPUT_ENABLED) != 0)  {
			for (i=0; i<3; ++i) {
				fromBufferInt16(oneTx, o, &s);
				imuData.linAcc[i] = ((float)s) / 1000.0f;	
				o = o + 2;
			}
		}	
	 
		if ((configReg & LPMS_PRESSURE_OUTPUT_ENABLED) != 0)  {
			fromBufferInt16(oneTx, o, &s);
			imuData.pressure = ((float)s) / 100.0f;	
			o = o + 2;
		}
		
		if ((configReg & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0)  {
			fromBufferInt16(oneTx, o, &s);
			imuData.altitude = ((float)s) / 10.0f;	
			o = o + 2;
		}

		if ((configReg & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0)  {
			fromBufferInt16(oneTx, o, &s);
			imuData.temperature = ((float)s) / 100.0f;	
			o = o + 2;
		}
		
		if ((configReg & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0)  {
			fromBufferInt16(oneTx, o, &s);
			imuData.hm.yHeave = ((float)s) / 100.0f;	
			o = o + 2;
		}
	} else {
		fromBufferFloat(oneTx, o, &currentTimestamp);
		o = o + 4;
		
		if ((configReg & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
			fromBufferVector3Float(oneTx, o, &r0, &r1, &r2);
			imuData.gRaw[0] = r0 * r2d;
			imuData.gRaw[1] = r1 * r2d;
			imuData.gRaw[2] = r2 * r2d;	
			o = o + 12;
		}
		
		if ((configReg & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
			fromBufferVector3Float(oneTx, o, &(imuData.aRaw[0]), &(imuData.aRaw[1]), &(imuData.aRaw[2]));	
			o = o + 12;
		}	

		if ((configReg & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
			fromBufferVector3Float(oneTx, o, &(imuData.bRaw[0]), &(imuData.bRaw[1]), &(imuData.bRaw[2]));	
			o = o + 12;
		}
		
		if ((configReg & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
			fromBufferVector3Float(oneTx, o, &r0, &r1, &r2);
			imuData.w[0] = r0 * r2d;
			imuData.w[1] = r1 * r2d;
			imuData.w[2] = r2 * r2d;	
			o = o + 12;
		}
		
		if ((configReg & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
			fromBufferVector4Float(oneTx, o, &(imuData.q[0]), &(imuData.q[1]), &(imuData.q[2]), &(imuData.q[3]));	
			o = o + 16;
		}	
			
		if ((configReg & LPMS_EULER_OUTPUT_ENABLED) != 0)  {
			fromBufferVector3Float(oneTx, o, &r0, &r1, &r2);
			imuData.r[0] = r0 * r2d;
			imuData.r[1] = r1 * r2d;
			imuData.r[2] = r2 * r2d;	
			o = o + 12;
		}	

		if ((configReg & LPMS_LINACC_OUTPUT_ENABLED) != 0)  {
			fromBufferVector3Float(oneTx, o, &(imuData.linAcc[0]), &(imuData.linAcc[1]), &(imuData.linAcc[2]));
			o = o + 12;
		}	
	 
		if ((configReg & LPMS_PRESSURE_OUTPUT_ENABLED) != 0)  {
			fromBufferFloat(oneTx, o, &imuData.pressure);
			o = o + 4;
		}
		
		if ((configReg & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0)  {
			fromBufferFloat(oneTx, o, &imuData.altitude);
			o = o + 4;
		}

		if ((configReg & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0)  {
			fromBufferFloat(oneTx, o, &imuData.temperature);
			o = o + 4;
		}
		
		if ((configReg & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0)  {
			fromBufferFloat(oneTx, o, &imuData.hm.yHeave);
			o = o + 4;
		}
	}
	
	imuData.timeStamp = currentTimestamp;
	
	lpmsPrintImuData(imuData);
	
	/* if (imuDataQueue.size() < 64) {
		imuDataQueue.push(imuData);
	} */

	return 1;
}

int parseFieldMapData(void)
{
	unsigned p, r, y;

	p = currentFieldMapPitch; 
	r = currentFieldMapRoll; 
	y = currentFieldMapYaw; 
	
	fromBufferVector3Float(oneTx, 0, &(configData->fieldMap[p][r][y].data[0]), &(configData->fieldMap[p][r][y].data[1]), &(configData->fieldMap[p][r][y].data[2]));
	
	return 1;
}

int lpmsParseFunction(void) 
{
	long l;
	long i0, i1, i2;
	int selectedData = 0;
	long a[64];
	int i;
	int doReceiveReset = 1;
	
	if (waitForAck == 1) {
		if (isAck() == 1) {
			ackReceived = 1;
						
			return 1;
		}

		if (isNack() == 1) {
			lpmsReceiveReset();
			
			return 0;
		}
	}
	
	switch (currentFunction) {
	case GET_CAN_CONFIGURATION:
		fromBufferInt32(oneTx, 0, &l);		
		
		if ((l & LPMS_CAN_FIXEDPOINT_MODE) != 0) {
			lpmsSetParameterInt(configData, PRM_CAN_POINT_MODE, SELECT_CAN_POINT_MODE_FIXED);	
		} else {
			lpmsSetParameterInt(configData, PRM_CAN_POINT_MODE, SELECT_CAN_POINT_MODE_FLOATING);
		}

		if ((l & LPMS_CAN_SEQUENTIAL_MODE) != 0) {
			lpmsSetParameterInt(configData, PRM_CAN_CHANNEL_MODE, SELECT_CAN_CHANNEL_MODE_SEQUENTIAL);
		} else {
			lpmsSetParameterInt(configData, PRM_CAN_CHANNEL_MODE, SELECT_CAN_CHANNEL_MODE_CANOPEN);		
		}
		
		selectedData = (l & 0xffff0000) >> 16;
		lpmsSetParameterInt(configData, PRM_CAN_START_ID, selectedData);
	break;
	
	case GET_CONFIG:
		// latestLatency = latencyTimer.measure() / 1000.0f;	
	
		if (fromBufferInt32(oneTx, 0, &l)) configReg = l;		
		
		if ((configReg & LPMS_GYR_THRES_ENABLED) != 0) {
			lpmsSetParameterInt(configData, PRM_GYR_THRESHOLD_ENABLED, SELECT_IMU_GYR_THRESH_ENABLED);
		} else {
			lpmsSetParameterInt(configData, PRM_GYR_THRESHOLD_ENABLED, SELECT_IMU_GYR_THRESH_DISABLED);
		}
		
		if ((configReg & LPMS_GYR_AUTOCAL_ENABLED) != 0) {
			lpmsSetParameterInt(configData, PRM_GYR_AUTOCALIBRATION, SELECT_GYR_AUTOCALIBRATION_ENABLED);
		} else {
			lpmsSetParameterInt(configData, PRM_GYR_AUTOCALIBRATION, SELECT_GYR_AUTOCALIBRATION_DISABLED);
		}
		
		if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_5HZ_ENABLED) {
			lpmsSetParameterInt(configData, PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_5HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_10HZ_ENABLED) {
			lpmsSetParameterInt(configData, PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_10HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_30HZ_ENABLED) {
			lpmsSetParameterInt(configData, PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_30HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_50HZ_ENABLED) {
			lpmsSetParameterInt(configData, PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_50HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_100HZ_ENABLED) {
			lpmsSetParameterInt(configData, PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_100HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_200HZ_ENABLED) {
			lpmsSetParameterInt(configData, PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_200HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_500HZ_ENABLED) {
			lpmsSetParameterInt(configData, PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_500HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_1000HZ_ENABLED) {
			lpmsSetParameterInt(configData, PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_1000HZ);	
		} else {
			lpmsSetParameterInt(configData, PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_100HZ);	
		}	

		if ((configReg & LPMS_CAN_BAUDRATE_MASK) == LPMS_CANBUS_BAUDRATE_125K_ENABLED) {
			lpmsSetParameterInt(configData, PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_125KBPS);
		} else if ((configReg & LPMS_CAN_BAUDRATE_MASK) == LPMS_CANBUS_BAUDRATE_250K_ENABLED) {
			lpmsSetParameterInt(configData, PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_250KBPS);
		} else if ((configReg & LPMS_CAN_BAUDRATE_MASK) == LPMS_CANBUS_BAUDRATE_500K_ENABLED) {
			lpmsSetParameterInt(configData, PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_500KBPS);
		} else {
			lpmsSetParameterInt(configData, PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_1000KBPS);
		}
		
		if ((configReg & LPMS_LPBUS_DATA_MODE_16BIT_ENABLED) != 0) {
			lpmsSetParameterInt(configData, PRM_LPBUS_DATA_MODE, SELECT_LPMS_LPBUS_DATA_MODE_16);
		} else {
			lpmsSetParameterInt(configData, PRM_LPBUS_DATA_MODE, SELECT_LPMS_LPBUS_DATA_MODE_32);
		}		
		
		if ((configReg & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_ACC_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_ACC_OUTPUT_ENABLED;	
		}

		if ((configReg & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_MAG_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_MAG_OUTPUT_ENABLED;	
		}		
		
		if ((configReg & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_GYRO_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_GYRO_OUTPUT_ENABLED;	
		}
		
		if ((configReg & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_QUAT_OUTPUT_ENABLED;	
		}

		if ((configReg & LPMS_EULER_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_EULER_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_EULER_OUTPUT_ENABLED;	
		}
		
		if ((configReg & LPMS_LINACC_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_LINACC_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_LINACC_OUTPUT_ENABLED;	
		}

		if ((configReg & LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_PRESSURE_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_PRESSURE_OUTPUT_ENABLED;	
		}

		if ((configReg & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED;	
		}		
		
		if ((configReg & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED;	
		}
		
		if ((configReg & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
		}
		
		if ((configReg & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED;
		}
		
		lpmsSetParameterInt(configData, PRM_SELECT_DATA, selectedData);
		
		if ((configReg & LPMS_HEAVEMOTION_ENABLED) != 0) {
			lpmsSetParameterInt(configData, PRM_HEAVEMOTION_ENABLED, SELECT_HEAVEMOTION_ENABLED);
		} else {
			lpmsSetParameterInt(configData, PRM_HEAVEMOTION_ENABLED, SELECT_HEAVEMOTION_DISABLED);
		}
		
		if ((configReg & LPMS_GAIT_TRACKING_ENABLED) != 0) {
			lpmsSetParameterInt(configData, PRM_GAIT_TRACKING_ENABLED, SELECT_GAIT_TRACKING_ENABLED);
		} else {
			lpmsSetParameterInt(configData, PRM_GAIT_TRACKING_ENABLED, SELECT_GAIT_TRACKING_DISABLED);
		}
	break;	
	
	case GET_SENSOR_DATA:
		parseSensorData();
		
		lpmsIoInterfaceReceivedNewData = 1;
		doReceiveReset = 0; // <- Careful, in case of GET_DATA don't reset anything to prevent overflow in stream mode.
		// if ((lpmsStatus & LPMS_STREAM_MODE) != 0) return 1;
	break;

	case GET_IMU_ID:
		if (fromBufferInt32(oneTx, 0, &l)) {
			lpmsSetParameterInt(configData, PRM_OPENMAT_ID, (int) l);
			imuId = l;
		}
	break;

	case GET_STATUS:
		if (fromBufferInt32(oneTx, 0, &l)) lpmsStatus = l;
	break;

	case GET_GYR_RANGE:
		if (fromBufferInt32(oneTx, 0, &l)) lpmsSetParameterInt(configData, PRM_GYR_RANGE, (int) l);
	break;

	case GET_ACC_RANGE:
		if (fromBufferInt32(oneTx, 0, &l)) lpmsSetParameterInt(configData, PRM_ACC_RANGE, (int) l);	
	break;

	case GET_MAG_RANGE:
		if (fromBufferInt32(oneTx, 0, &l)) lpmsSetParameterInt(configData, PRM_MAG_RANGE, (int) l);			
	break;
	
	case GET_FILTER_MODE:
		fromBufferInt32(oneTx, 0, &l);
		switch(l) {
		case LPMS_FILTER_GYR:
			lpmsSetParameterInt(configData, PRM_FILTER_MODE, SELECT_FM_GYRO_ONLY);
		break;
		
		case LPMS_FILTER_GYR_ACC:
			lpmsSetParameterInt(configData, PRM_FILTER_MODE, SELECT_FM_GYRO_ACC);
		break;

		case LPMS_FILTER_GYR_ACC_MAG:
			lpmsSetParameterInt(configData, PRM_FILTER_MODE, SELECT_FM_GYRO_ACC_MAG);
		break;

		case LPMS_FILTER_ACC_MAG:
			lpmsSetParameterInt(configData, PRM_FILTER_MODE, SELECT_FM_ACC_MAG);
		break;
		
		case LPMS_FILTER_GYR_ACC_EULER:
			lpmsSetParameterInt(configData, PRM_FILTER_MODE, SELECT_FM_GYR_ACC_EULER);
		break;
		}
	break;
	
	case GET_FILTER_PRESET:
		fromBufferInt32(oneTx, 0, &l);
		switch(l) {
		case LPMS_FILTER_PRM_SET_1:
			lpmsSetParameterInt(configData, PRM_PARAMETER_SET, SELECT_IMU_SLOW);				
		break;

		case LPMS_FILTER_PRM_SET_2:
			lpmsSetParameterInt(configData, PRM_PARAMETER_SET, SELECT_IMU_MEDIUM);				
		break;
		
		case LPMS_FILTER_PRM_SET_3:
			lpmsSetParameterInt(configData, PRM_PARAMETER_SET, SELECT_IMU_FAST);				
		break;	
		
		case LPMS_FILTER_PRM_SET_4:
			lpmsSetParameterInt(configData, PRM_PARAMETER_SET, SELECT_IMU_DYNAMIC);				
		break;	
		}
	break;
	
	case GET_HARD_IRON_OFFSET:
		fromBufferVector3Float(oneTx, 0, &(configData->hardIronOffset.data[0]), &(configData->hardIronOffset.data[1]), &(configData->hardIronOffset.data[2]));
	break;
	
	case GET_ACC_BIAS:
		fromBufferVector3Float(oneTx, 0, &(configData->accBias.data[0]), &(configData->accBias.data[1]), &(configData->accBias.data[2]));
	break;
	
	case GET_SOFT_IRON_MATRIX:
		for (i=0; i<3; i++) {
			fromBufferVector3Float(oneTx, i*12, &(configData->softIronMatrix.data[i][0]), &(configData->softIronMatrix.data[i][1]), &(configData->softIronMatrix.data[i][2]));
		}
	break;
	
	case GET_ACC_ALIGN_MATRIX:
		for (i=0; i<3; i++) {
			fromBufferVector3Float(oneTx, i*12, &(configData->misalignMatrix.data[i][0]), &(configData->misalignMatrix.data[i][1]), &(configData->misalignMatrix.data[i][2]));
		}
	break;

	case GET_FIELD_ESTIMATE:
		fromBufferFloat(oneTx, 0, &(configData->fieldRadius));
	break;
	
	case GET_FIRMWARE_VERSION:
		fromBufferVector3Int32(oneTx, 0, &i0, &i1, &i2);
		configData->firmwareVersionDig0 = i0;
		configData->firmwareVersionDig1 = i1;
		configData->firmwareVersionDig2 = i2;
				
		// configData->firmwareVersion = static_cast<ostringstream*>(&(ostringstream() << i2))->str() + std::string(".") + static_cast<ostringstream*>(&(ostringstream() << i1))->str() + std::string(".") + static_cast<ostringstream*>(&(ostringstream() << i0))->str();
	break;

	case GET_GYR_ALIGN_BIAS:
		fromBufferVector3Float(oneTx, 0, &(configData->gyrAlignmentBias.data[0]), &(configData->gyrAlignmentBias.data[1]), &(configData->gyrAlignmentBias.data[2]));
	break;
	
	case GET_GYR_ALIGN_MATRIX:
		for (i=0; i<3; i++) {
			fromBufferVector3Float(oneTx, i*12, &(configData->gyrMisalignMatrix.data[i][0]), &(configData->gyrMisalignMatrix.data[i][1]), &(configData->gyrMisalignMatrix.data[i][2]));
		}		
	break;
	
	case GET_MAG_ALIGNMENT_BIAS:
		fromBufferVector3Float(oneTx, 0, &(configData->magMAlignmentBias.data[0]), &(configData->magMAlignmentBias.data[1]), &(configData->magMAlignmentBias.data[2]));
	break;
	
	case GET_MAG_ALIGNMENT_MATRIX:
		for (i=0; i<3; i++) {
			fromBufferVector3Float(oneTx, i*12, &(configData->magMAlignmentMatrix.data[i][0]), &(configData->magMAlignmentMatrix.data[i][1]), &(configData->magMAlignmentMatrix.data[i][2]));
		}		
	break;
	
	case GET_MAG_REFERENCE:
		fromBufferVector3Float(oneTx, 0, &(configData->magReference.data[0]), &(configData->magReference.data[1]), &(configData->magReference.data[2]));
	break;
	
	case GET_RAW_DATA_LP:
		fromBufferInt32(oneTx, 0, &l);
		switch(l) {
		case LPMS_LP_OFF:
			lpmsSetParameterInt(configData, PRM_LOW_PASS, SELECT_LPMS_LP_OFF);				
		break;

		case LPMS_LP_01:
			lpmsSetParameterInt(configData, PRM_LOW_PASS, SELECT_LPMS_LP_01);				
		break;
		
		case LPMS_LP_005:
			lpmsSetParameterInt(configData, PRM_LOW_PASS, SELECT_LPMS_LP_005);				
		break;	
		
		case LPMS_LP_001:
			lpmsSetParameterInt(configData, PRM_LOW_PASS, SELECT_LPMS_LP_001);				
		break;	
		
		case LPMS_LP_0005:
			lpmsSetParameterInt(configData, PRM_LOW_PASS, SELECT_LPMS_LP_0005);				
		break;

		case LPMS_LP_0001:
			lpmsSetParameterInt(configData, PRM_LOW_PASS, SELECT_LPMS_LP_0001);				
		break;
		}
	break;
	
	case GET_CAN_MAPPING:
		fromBufferInt32Array(oneTx, a, 16);	
		lpmsSetParameterIntArray(configData, PRM_CAN_MAPPING, (int *) a);		
	break;
	
	case GET_CAN_HEARTBEAT:
		fromBufferInt32(oneTx, 0, &l);	
		lpmsSetParameterInt(configData, PRM_CAN_HEARTBEAT, (int) l);
	break;
	
	case GET_LIN_ACC_COMP_MODE:
		fromBufferInt32(oneTx, 0, &l);	
		switch(l) {
		case LPMS_LIN_ACC_COMP_MODE_OFF:
			lpmsSetParameterInt(configData, PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_OFF);				
		break;

		case LPMS_LIN_ACC_COMP_MODE_WEAK:
			lpmsSetParameterInt(configData, PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_WEAK);				
		break;
		
		case LPMS_LIN_ACC_COMP_MODE_MEDIUM:
			lpmsSetParameterInt(configData, PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_MEDIUM);				
		break;	
		
		case LPMS_LIN_ACC_COMP_MODE_STRONG:
			lpmsSetParameterInt(configData, PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_STRONG);				
		break;	

		case LPMS_LIN_ACC_COMP_MODE_ULTRA:
			lpmsSetParameterInt(configData, PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_ULTRA);				
		break;	
		}
	break;
	
	case GET_CENTRI_COMP_MODE:
		fromBufferInt32(oneTx, 0, &l);	
		switch(l) {
		case LPMS_CENTRI_COMP_MODE_OFF:
			lpmsSetParameterInt(configData, PRM_CENTRI_COMP_MODE, SELECT_LPMS_CENTRI_COMP_MODE_OFF);				
		break;

		case LPMS_CENTRI_COMP_MODE_ON:
			lpmsSetParameterInt(configData, PRM_CENTRI_COMP_MODE, SELECT_LPMS_CENTRI_COMP_MODE_ON);				
		break;	
		}
	break;

	default:
		doReceiveReset = 0;
	break;
	}

	if (doReceiveReset == 1) lpmsReceiveReset();

	return 1;
}

int lpmsGetLatestImuData(ImuData *id) 
{
	int r = lpmsIoInterfaceReceivedNewData;

	*id = imuData;

	lpmsIoInterfaceReceivedNewData = 0;
	
	/* if (imuDataQueue.empty() == 1) return 0;
	
	*id = imuDataQueue.front();
	imuDataQueue.pop(); */
	
	return r;
}

int lpmsCheckState(void)
{
	// lpmsParseModbusByte();

	if (waitForAck == 1 && ackReceived == 0) {	
		/* if (ackTimer.measure() > ACK_MAX_TIME) {
			if (resendIndex < MAX_COMMAND_RESEND) {
				ackTimer.reset();
				uploadTimer.reset();
				++resendIndex;
				sendModbusData(imuId, currentState, cLength, cBuffer);
				
				LOGV("[LpmsIoInterface] ACK timeout error. Resending command: %d\n", resendIndex);
				
				return 1;
			} else {			
				currentState = IDLE_STATE;
				waitForAck = 0;
				waitForData = 0;				
				ackReceived = 0;
				ackTimeout = 0;
				isOpen = 0;
			
				LOGV("[LpmsIoInterface] ACK timeout error. Resetting send queue.\n");
			
				if (ifs.is_open() == 1) ifs.close();
			
				return 0;
			}
		} */
	} 
	
	if (waitForData == 1 && dataReceived == 0) {
		/* if (ackTimer.measure() > ACK_MAX_TIME) {
			if (resendIndex < MAX_COMMAND_RESEND) {
				ackTimer.reset();
				uploadTimer.reset();
				++resendIndex;
				sendModbusData(imuId, currentState, cLength, cBuffer);
				
				LOGV("[LpmsIoInterface] ACK timeout error. Resending command: %d\n", resendIndex);
				
				return 1;
			} else {			
				currentState = IDLE_STATE;
				waitForAck = 0;
				waitForData = 0;
				ackReceived = 0;
				ackTimeout = 0;
				isOpen = 0;
			
				LOGV("[LpmsIoInterface] ACK timeout error. Resetting send queue.\n");
			
				return 0;
			}
		} */
	}

	if (waitForAck == 1 && ackReceived == 1) {
		switch (currentState) {
			case UPDATE_FIRMWARE:
				lpmsHandleFirmwareFrame();
			break;
			
			case UPDATE_IAP:
				lpmsHandleIAPFrame();
			break;
			
			case SET_CAN_STREAM_FORMAT:
				lpmsReceiveReset();
				lpmsZeroImuData(&imuData);
			break;
			
			default:
				lpmsReceiveReset();
			break;
		}
	}
	
	return 1;
}

int lpmsIsWaitForData(void)
{
	return waitForData;
}

int lpmsIsWaitForAck(void)
{
	return waitForAck;
}

int lpmsIsCalibrating(void)
{
	if ((lpmsStatus & (LPMS_GYR_CALIBRATION_RUNNING | 
		LPMS_MAG_CALIBRATION_RUNNING | LPMS_REF_CALIBRATION_RUNNING)) != 0) {
		return 1;
	}
	
	return 0;
}

int lpmsIsError(void)
{
	if (((lpmsStatus & (LPMS_GYR_INIT_FAILED | LPMS_ACC_INIT_FAILED | 
		LPMS_MAG_INIT_FAILED | LPMS_PRESSURE_INIT_FAILED | LPMS_GYR_UNRESPONSIVE |
		LPMS_ACC_UNRESPONSIVE | LPMS_MAG_UNRESPONSIVE | LPMS_FLASH_WRITE_FAILED |LPMS_SET_BAUDRATE_FAILED | LPMS_SET_BROADCAST_FREQ_FAILED)) != 0)) {
		return 1;
	}
	
	return 0;
}

CalibrationData *lpmsGetConfigData(void)
{
	return configData;
}

int lpmsGetCurrentState(void)
{
	return currentState;
}



/***********************************************************************
** FIRMWARE / IAP
***********************************************************************/
int lpmsStartUploadFirmware(char* fn)
{
	/* int f = 0;
	long long l;
	unsigned long r;
	
	if (configData->deviceType == DEVICE_LPMS_BLE) {
		firmwarePageSize = FIRMWARE_PACKET_LENGTH_LPMS_BLE;
	} else {
		firmwarePageSize = FIRMWARE_PACKET_LENGTH;
	}
	
	if (ifs.is_open() == 1) ifs.close();

	ifs.clear();	
	ifs.open(fn.c_str(), std::ios::binary);
		
	if (ifs.is_open() == 1) {
		f = 1;
		LOGV("[LpmsIoInterface] Firmware file %s opened.\n", fn.c_str());
	} else {
		LOGV("[LpmsIoInterface] Could not open firmware file %s\n", fn.c_str());
		f = 0;
	
		return f;
	}
	
	ifs.seekg(0, ios::end);
	l = ifs.tellg();
	ifs.seekg(0, ios::beg);
	LOGV("[LpmsIoInterface] Firmware filesize: %d\n", l);
	
	firmwarePages = l / firmwarePageSize;
	r = l % firmwarePageSize;
	
	if (r > 0) ++firmwarePages;
	
	LOGV("[LpmsIoInterface] Firmware pages: %d\n", firmwarePages);	
	LOGV("[LpmsIoInterface] Firmware remainder: %d\n", r);
	
	cBuffer[0] = firmwarePages & 0xff;
	cBuffer[1] = (firmwarePages >> 8) & 0xff;
	cBuffer[2] = (firmwarePages >> 16) & 0xff;
	cBuffer[3] = (firmwarePages >> 24) & 0xff;
	
	LOGV("[LpmsIoInterface] Firmware packets to be sent: %d\n", firmwarePages);
	
	cLength = 4;
	sendModbusData(imuId, UPDATE_FIRMWARE, 4, (unsigned char *)cBuffer);	
	
	currentState = UPDATE_FIRMWARE;
	waitForAck = 1;
	ackReceived = 0;
	ackTimeout = 0;
	ackTimer.reset();

	pCount = 0;	
	uploadTimer.reset();
	
	return f; */
	
	return 0;
}

int lpmsCheckUploadTimeout(void)
{
	/* if (currentState == UPDATE_FIRMWARE) {
		if (uploadTimer.measure() > MAX_UPLOAD_TIME) {	
			currentState = IDLE_STATE;
		
			waitForAck = 0;
			ackReceived = 0;	

			ifs.close();
			
			LOGV("[LpmsIoInterface] Firmware upload failed. Please reconnect sensor and retry.\n");

			return 0;
		}
	} */
	
	return 1;
}

int lpmsHandleFirmwareFrame(void)
{
	/* uploadTimer.reset();
	
	if (ifs.is_open() == 0) {
		currentState = IDLE_STATE;
		waitForAck = 0;
		ackReceived = 0;
		
		ifs.close();		
		
		return 0;
	}

	if (ifs.eof() == 1 || firmwarePages == pCount) {
		currentState = IDLE_STATE;
		waitForAck = 0;
		ackReceived = 0;	

		ifs.close();
		
		LOGV("[LpmsIoInterface] Firmware upload finished. Now writing to flash. Please DO NOT detach the power from the device for 15s.\n");
		
		return 1;
	}

	LOGV("[LpmsIoInterface] Firmware sending packet %d\n", pCount);
	++pCount;

	for (unsigned i=0; i < firmwarePageSize; i++) cBuffer[i] = (char) 0xff;
	ifs.read((char *)cBuffer, firmwarePageSize);
	cLength = firmwarePageSize;
	sendModbusData(imuId, UPDATE_FIRMWARE, firmwarePageSize, (unsigned char *)cBuffer);

	ackTimeout = 0;
	ackTimer.reset();
	dataTimeout = 0;
	resendIndex = 0;

	currentState = UPDATE_FIRMWARE;
	waitForAck = 1;
	ackReceived = 0; */	
	
	return 1;
}

int lpmsStartUploadIap(char* fn)
{
	/* int f = 0;

	if (configData->deviceType == DEVICE_LPMS_BLE) {
		firmwarePageSize = FIRMWARE_PACKET_LENGTH_LPMS_BLE;
	} else {
		firmwarePageSize = FIRMWARE_PACKET_LENGTH;
	}	
	
	if (ifs.is_open() == 1) ifs.close();
	
	ifs.clear();	
	ifs.open(fn.c_str(), std::ios::binary);
		
	if (ifs.is_open() == 1) {
		f = 1;
		LOGV("[LpmsIoInterface] IAP file %s opened.\n", fn.c_str());
	} else {
		LOGV("[LpmsIoInterface] Could not open IAP file %s\n", fn.c_str());	
		f = 0;
	}
	
	ifs.seekg(0, ios::end);
	long long l = ifs.tellg();
	ifs.seekg(0, ios::beg);
	LOGV("[LpmsIoInterface] IAP filesize: %d\n", l);
	
	firmwarePages = l / firmwarePageSize;
	unsigned long r = (long) (l % firmwarePageSize);
	
	if (r > 0) ++firmwarePages;
	
	LOGV("[LpmsIoInterface] IAP pages: %d\n", firmwarePages);
	LOGV("[LpmsIoInterface] IAP remainder: %d\n", r);
	
	cBuffer[0] = firmwarePages & 0xff;
	cBuffer[1] = (firmwarePages >> 8) & 0xff;
	cBuffer[2] = (firmwarePages >> 16) & 0xff;
	cBuffer[3] = (firmwarePages >> 24) & 0xff;
	
	LOGV("[LpmsIoInterface] IAP packets to be sent: %d\n", firmwarePages);
	
	cLength = 4;
	sendModbusData(imuId, UPDATE_IAP, 4, (unsigned char *)cBuffer);	
	
	currentState = UPDATE_IAP;
	waitForAck = 1;
	ackReceived = 0;
	ackTimeout = 0;
	ackTimer.reset();

	pCount = 0;
	uploadTimer.reset();
	
	return f; */
	
	return 0;
}	

int lpmsHandleIAPFrame(void)
{
	/* uploadTimer.reset();	
	
	if (ifs.is_open() == 0) {
		currentState = IDLE_STATE;
		waitForAck = 0;
		ackReceived = 0;
			
		ifs.close();	
		
		return 0;
	}

	if (ifs.eof() == 1 || firmwarePages == pCount) {
		currentState = IDLE_STATE;
		waitForAck = 0;
		ackReceived = 0;

		ifs.close();
		
		LOGV("[LpmsIoInterface] IAP upload finished\n");
		
		return 1;		
	}
	
	LOGV("[LpmsIoInterface] Sending IAP packet %d\n", pCount);
	++pCount;		
	
	for (unsigned i=0; i < firmwarePageSize; i++) cBuffer[i] = (char) 0xff;
	ifs.read((char *)cBuffer, firmwarePageSize);
	cLength = firmwarePageSize;
	sendModbusData(imuId, UPDATE_IAP, firmwarePageSize, (unsigned char *)cBuffer);
	
	ackTimeout = 0;
	dataTimeout = 0;
	resendIndex = 0;
	ackTimer.reset();
	
	currentState = UPDATE_IAP;
	waitForAck = 1;
	ackReceived = 0; */	
		
	return 1;
}

int lpmsGetUploadProgress(int *p)
{
	if (firmwarePages > 0) {
		*p = pCount * 100 / (int) firmwarePages;
	} else {
		*p = 0;
	}

	if (lpmsCheckUploadTimeout() == 0) return 0;
	
	return 1;
}



/***********************************************************************
** SET / GET COMMANDS
***********************************************************************/
int lpmsSelectData(long p) 
{	
	uint32_t v = 0;
	
	if ((p & SELECT_LPMS_ACC_OUTPUT_ENABLED) != 0) {
		v |= LPMS_ACC_RAW_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_ACC_RAW_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_MAG_OUTPUT_ENABLED) != 0) {
		v |= LPMS_MAG_RAW_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_MAG_RAW_OUTPUT_ENABLED;
	}	
	
	if ((p & SELECT_LPMS_GYRO_OUTPUT_ENABLED) != 0) {
		v |= LPMS_GYR_RAW_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_GYR_RAW_OUTPUT_ENABLED;
	}	
	
	if ((p & SELECT_LPMS_QUAT_OUTPUT_ENABLED) != 0) {
		v |= LPMS_QUAT_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_QUAT_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_EULER_OUTPUT_ENABLED) != 0) {
		v |= LPMS_EULER_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_EULER_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {
		v |= LPMS_PRESSURE_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_PRESSURE_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED) != 0) {
		v |= LPMS_ALTITUDE_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_ALTITUDE_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0) {
		v |= LPMS_TEMPERATURE_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_TEMPERATURE_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_LINACC_OUTPUT_ENABLED) != 0) {
		v |= LPMS_LINACC_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_LINACC_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
		v |= LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
		v |= LPMS_HEAVEMOTION_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_HEAVEMOTION_OUTPUT_ENABLED;
	}
	
	modbusSetInt32(SET_TRANSMIT_DATA, v);
	
	return 1;
}

int lpmsGetMode(void)
{
	if ((lpmsStatus & LPMS_COMMAND_MODE) != 0) {
			return SELECT_LPMS_MODE_COMMAND;
		} else if ((lpmsStatus & LPMS_STREAM_MODE) != 0) {
			return SELECT_LPMS_MODE_STREAM;
		} else if ((lpmsStatus & LPMS_SLEEP_MODE) != 0) {
			return SELECT_LPMS_MODE_SLEEP;
		}

	return currentMode;
}

int lpmsSetCommandMode(void) 
{	
	int r;

	lpmsStatus |= LPMS_COMMAND_MODE;
	lpmsStatus &= ~LPMS_STREAM_MODE;
	lpmsStatus &= ~LPMS_SLEEP_MODE;
	
	r = modbusSetNone(GOTO_COMMAND_MODE);
	
	return r;
}

int lpmsSetStreamMode(void) 
{	
	int r;

	lpmsStatus &= ~LPMS_COMMAND_MODE;
	lpmsStatus |= LPMS_STREAM_MODE;
	lpmsStatus &= ~LPMS_SLEEP_MODE;	
	
	r = modbusSetNone(GOTO_STREAM_MODE);

	return r;
}
	
int lpmsSetSleepMode(void) 
{	
	int r;

	lpmsStatus &= ~LPMS_COMMAND_MODE;
	lpmsStatus &= ~LPMS_STREAM_MODE;
	lpmsStatus |= LPMS_SLEEP_MODE;	
	
	r = modbusSetNone(GOTO_SLEEP_MODE);

	return r;
}

int lpmsRestoreFactoryValues(void)
{
	return modbusSetNone(RESTORE_FACTORY_VALUE);
}

int lpmsSetSelfTest(long v)
{
	return modbusSetInt32(SELF_TEST, v);
}

int lpmsSetImuId(long v)
{
	int f;
	
	f = modbusSetInt32(SET_IMU_ID, v);
	imuId = v;

	return f;
}

int lpmsSetStreamFrequency(long v)
{
	return modbusSetInt32(SET_STREAM_FREQ, v);
}

int lpmsStartGyrCalibration(void)
{
	return modbusSetNone(START_GYR_CALIBRA);
}

int lpmsSetGyrRange(long v)
{
	return modbusSetInt32(SET_GYR_RANGE, v);
}

int lpmsSetMagRange(long v)
{
	return modbusSetInt32(SET_MAG_RANGE, v);
}

int lpmsSetAccRange(long v)
{
	return modbusSetInt32(SET_ACC_RANGE, v);
}

int lpmsSetAccBias(LpVector3f v)
{
	return modbusSetVector3f(SET_ACC_BIAS, v);
}

int lpmsEnableGyrThres(long v)
{
	return modbusSetInt32(ENABLE_GYR_THRES, v);
}

int lpmsEnableGyrAutocalibration(long v)
{
	return modbusSetInt32(ENABLE_GYR_AUTOCAL, v);
}

int lpmsSetFilterMode(long v)
{
	return modbusSetInt32(SET_FILTER_MODE, v);
}

int lpmsSetFilterPreset(long v)
{
	return modbusSetInt32(SET_FILTER_PRESET, v);
}

int lpmsSetCanStreamFormat(long v)
{
	return modbusSetInt32(SET_CAN_STREAM_FORMAT, v);
}

int lpmsSetCanBaudrate(long v)
{
	return modbusSetInt32(SET_CAN_BAUDRATE, v);
}

int lpmsSetFieldEstimate(float v)
{
	return modbusSetFloat(SET_FIELD_ESTIMATE, v);
}

int lpmsGetConfig(void)
{
	// latencyTimer.reset();

	return modbusGet(GET_CONFIG);
}

int lpmsGetImuId(void)
{
	return modbusGet(GET_IMU_ID);
}

int lpmsGetStatus(void)
{
	return modbusGet(GET_STATUS);
}

int lpmsGetGyrRange(void)
{
	return modbusGet(GET_GYR_RANGE);
}

int lpmsGetAccRange(void)
{
	return modbusGet(GET_ACC_RANGE);
}

int lpmsGetMagRange(void)
{
	return modbusGet(GET_MAG_RANGE);
}	

int lpmsGetAccBias(void)
{
	return modbusGet(GET_ACC_BIAS);
}

int lpmsGetSensorData(void)
{
	return modbusGet(GET_SENSOR_DATA);
}

int lpmsGetFilterMode(void)
{
	return modbusGet(GET_FILTER_MODE);
}

int lpmsGetFilterPreset(void)
{
	return modbusGet(GET_FILTER_PRESET);
}

int lpmsGetFieldEstimate(void)
{
	return modbusGet(GET_FIELD_ESTIMATE);
}

int lpmsWriteRegisters(void)
{
	return modbusSetNone(WRITE_REGISTERS);
}

int lpmsGetHardIronOffset(void)
{
	return modbusGet(GET_HARD_IRON_OFFSET);
}

int lpmsGetSoftIronMatrix(void)
{
	return modbusGet(GET_SOFT_IRON_MATRIX);
}

int lpmsSetHardIronOffset(LpVector3f v)
{
	return modbusSetVector3f(SET_HARD_IRON_OFFSET, v);
}

int lpmsSetSoftIronMatrix(LpMatrix3x3f m)
{
	return modbusSetMatrix3x3f(SET_SOFT_IRON_MATRIX, m);
}

int lpmsSetAccAlignment(LpMatrix3x3f m)
{
	return modbusSetMatrix3x3f(SET_ACC_ALIGN_MATRIX, m);
}

int lpmsGetAccAlignment(void)
{
	return modbusGet(GET_ACC_ALIGN_MATRIX);
}

int lpmsGetFirmwareVersion(void)
{
	return modbusGet(GET_FIRMWARE_VERSION);
}

int lpmsSetGyrAlignment(LpMatrix3x3f m)
{
	return modbusSetMatrix3x3f(SET_GYR_ALIGN_MATRIX, m);
}

int lpmsSetGyrAlignmentBias(LpVector3f v)
{
	return modbusSetVector3f(SET_GYR_ALIGN_BIAS, v);
}

int lpmsGetGyrAlignment(void)
{
	return modbusGet(GET_GYR_ALIGN_MATRIX);
}

int lpmsGetGyrAlignmentBias(void)
{
	return modbusGet(GET_GYR_ALIGN_BIAS);
}

long lpmsGetConfigReg(void) {
	return configReg;
}

int lpmsGetRawDataLpFilter(void)
{
	return modbusGet(GET_RAW_DATA_LP);	
}

int lpmsSetRawDataLpFilter(int v)
{
	return modbusSetInt32(SET_RAW_DATA_LP, v);
}

int lpmsGetCanMapping(void)
{
	return modbusGet(GET_CAN_MAPPING);	
}

int lpmsSetCanMapping(int *a)
{	
	return modbusSetInt32Array(SET_CAN_MAPPING, (long *) a, 16);
}

int lpmsGetCanHeartbeat(void)
{
	return modbusGet(GET_CAN_HEARTBEAT);	
}

int lpmsSetCanHeartbeat(int v)
{
	return modbusSetInt32(SET_CAN_HEARTBEAT, v);
}

int lpmsSetTimestamp(float v)
{
	uint32_t i, j;
	uint32_t m = 0xff;
	unsigned char buffer[4];		

	i = conFtoI(v);	

	for (j=0; j<4; j++) {
		buffer[j] = (unsigned char) (i & 0xff);
		i = i / 256;
	}

	lpmsSendModbusData(imuId, RESET_TIMESTAMP, 4, buffer);

	return 1;
}

int lpmsGetLinAccCompMode(void)
{
	return modbusGet(GET_LIN_ACC_COMP_MODE);
}

int lpmsSetLinAccCompMode(int v)
{
	return modbusSetInt32(SET_LIN_ACC_COMP_MODE, v);
}

int lpmsGetCentriCompMode(void)
{
	return modbusGet(GET_CENTRI_COMP_MODE);
}

int lpmsSetCentriCompMode(int v)
{
	return modbusSetInt32(SET_CENTRI_COMP_MODE, v);
}

int lpmsGetCanConfiguration(void)
{
	return modbusGet(GET_CAN_CONFIGURATION);
}

int lpmsSetCanChannelMode(int v)
{
	return modbusSetInt32(SET_CAN_CHANNEL_MODE, v);
}

int lpmsSetCanPointMode(int v)
{
	return modbusSetInt32(SET_CAN_POINT_MODE, v);
}

int lpmsSetCanStartId(int v)
{
	return modbusSetInt32(SET_CAN_START_ID, v);
}

int lpmsSetLpBusDataMode(int v)
{
	return modbusSetInt32(SET_LPBUS_DATA_MODE, v);
}

int lpmsSetMagAlignmentMatrix(LpMatrix3x3f m)
{
	return modbusSetMatrix3x3f(SET_MAG_ALIGNMENT_MATRIX, m);
}

int lpmsSetMagAlignmentBias(LpVector3f v)
{
	return modbusSetVector3f(SET_MAG_ALIGNMENT_BIAS, v);
}

int lpmsSetMagReference(LpVector3f v)
{
	return modbusSetVector3f(SET_MAG_REFRENCE, v);
}

int lpmsGetMagReference(void)
{
	return modbusGet(GET_MAG_REFERENCE);
}

int lpmsGetMagAlignmentMatrix(void)
{
	return modbusGet(GET_MAG_ALIGNMENT_MATRIX);
}

int lpmsGetMagAlignmentBias(void)
{
	return modbusGet(GET_MAG_ALIGNMENT_BIAS);
}

int lpmsSetOrientationOffset(void)
{
	return modbusSetNone(SET_ORIENTATION_OFFSET);
}

int lpmsresetOrientationOffset(void)
{
	return modbusSetNone(RESET_ORIENTATION_OFFSET);
}



/***********************************************************************
** DATA CONVERSION
***********************************************************************/
uint32_t conFtoI(float f)
{
	float2uint f2int;
	f2int.fp = f;
	
	return f2int.up;
}

float conItoF(uint32_t v)
{
	float2uint f2int;
	f2int.up = v;
	
	return f2int.fp;
}	

int modbusSetNone(unsigned command) 
{
	int r;

	lpmsReceiveReset();

	cLength = 0;
	r = lpmsSendModbusData(imuId, command, 0, cBuffer);
	
	currentState = command;
	waitForAck = 1;
	waitForData = 0;
	ackReceived = 0;
	ackTimeout = 0;
	
	// ackTimer.reset();
	
	return r;
}

int modbusGet(unsigned command) 
{
	int r;

	lpmsReceiveReset();

	cLength = 0;
	r = lpmsSendModbusData(imuId, command, 0, cBuffer);
	
	currentState = command;
	
	waitForData = 1;
	waitForAck = 0;	
	dataReceived = 0;
	dataTimeout = 0;
	
	// ackTimer.reset();	
	
	return r;
}

int modbusGetMultiUint32(unsigned command, uint32_t *v, int n) 
{
	int r, j, i;
	uint32_t t;
	
	lpmsReceiveReset();

	for (j=0; j<n; ++j) {
		t = v[j];
		for (i=0; i<4; ++i) {
			cBuffer[j*4+i] = t & 0xff;
			t = t >> 8;
		}
	}
	
	cLength = n*4;
	r = lpmsSendModbusData(imuId, command, n*4, cBuffer);
	
	currentState = command;
	waitForData = 1;
	ackReceived = 0;
	ackTimeout = 0;
	
	// ackTimer.reset();
	
	return r;
}

int modbusSetInt32(unsigned command, long v)
{
	int r, i;	
	
	lpmsReceiveReset();

	for (i=0; i<4; ++i) {
		cBuffer[i] = (unsigned char) (v & 0xff);
		v = v >> 8;
	}
	
	cLength = 4;
	r = lpmsSendModbusData(imuId, command, 4, cBuffer);
	
	currentState = command;
	waitForAck = 1;
	ackReceived = 0;
	ackTimeout = 0;
	
	// ackTimer.reset();
	
	return r;
}

int modbusSetInt32Array(unsigned command, long *v, int length)
{
	int r, j, i;
	
	lpmsReceiveReset();

	for (j=0; j<length; ++j) {
		for (i=0; i<4; ++i) {
			cBuffer[j*4+i] = (unsigned char) (v[j] & 0xff);
			v[j] = v[j] >> 8;
		}
	}
	
	cLength = 4*length;
	r = lpmsSendModbusData(imuId, command, 4*length, cBuffer);
	
	currentState = command;
	waitForAck = 1;
	ackReceived = 0;
	ackTimeout = 0;
	
	// ackTimer.reset();
	
	return r;
}

int modbusSetVector3Int32(unsigned command, long x, long y, long z)
{
	int r, i, j;
	long v[3] = { x, y, z };

	lpmsReceiveReset();
	
	for (i=0; i<3; ++i) {	
		for (j=3; j>=0; --j) {
			cBuffer[i*4+j] = (unsigned char) (v[i] & 0xff);
			v[i] = v[i] / 256;
		}
	}
	
	cLength = 12;
	r = lpmsSendModbusData(imuId, command, 12, cBuffer);
	
	currentState = command;
	waitForAck = 1;
	ackReceived = 0;
	ackTimeout = 0;
	
	// ackTimer.reset();
	
	return r;
}

int modbusSetFloat(unsigned command, float v)
{
	uint32_t i;
	uint32_t m = 0xff;
	int r, j;		

	lpmsReceiveReset();

	i = conFtoI(v);

	for (j=0; j<4; j++) {
		cBuffer[j] = (unsigned char) (i & 0xff);
		i = i / 256;
	}

	cLength = 4;
	r = lpmsSendModbusData(imuId, command, 4, cBuffer);
	
	currentState = command;
	waitForAck = 1;
	ackReceived = 0;
	ackTimeout = 0;
	
	// ackTimer.reset();
	
	return r;		
}

int modbusSetVector3Float(unsigned command, float x, float y, float z)
{
	uint32_t i;
	uint32_t m = 0xff;
	float v[3] = { x, y, z };
	int r, j, k;	
	
	lpmsReceiveReset();

	for (j=0; j<3; ++j) {	
		i = conFtoI(v[j]);
		for (k=3; k>=0; --k) {
			cBuffer[j*4+k] = (unsigned char) (i & 0xff);
			i = i / 256;
		}
	}

	cLength = 12;
	r = lpmsSendModbusData(imuId, command, 12, cBuffer);
	
	currentState = command;
	waitForAck = 1;
	ackReceived = 0;
	ackTimeout = 0;	
	
	// ackTimer.reset();
	
	return r;		
}

int modbusSetVector3f(unsigned command, LpVector3f v)
{
	uint32_t i;	
	int r, j, k;	
	
	lpmsReceiveReset();

	for (j=0; j<3; j++) {	
		i = conFtoI(v.data[j]);
		for (k=0; k<4; k++) {
			cBuffer[j*4+k] = (unsigned char) (i & 0xff);
			i = i / 256;
		}
	}

	cLength = 12;
	r = lpmsSendModbusData(imuId, command, 12, cBuffer);
	
	currentState = command;
	waitForAck = 1;
	ackReceived = 0;
	ackTimeout = 0;	
	
	// ackTimer.reset();
	
	return r;		
}

int modbusSetMatrix3x3f(unsigned command, LpMatrix3x3f m)
{
	uint32_t i;
	int r, j, l, k;	
	
	lpmsReceiveReset();

	for (j=0; j<3; j++) {	
		for (l=0; l<3; l++) {
			i = conFtoI(m.data[j][l]);
			for (k=0; k<4; k++) {
				cBuffer[j*3*4+l*4+k] = (unsigned char) (i & 0xff);
				i = i / 256;
			}
		}
	}

	cLength = 36;
	r = lpmsSendModbusData(imuId, command, 36, cBuffer);
	
	currentState = command;
	waitForAck = 1;
	ackReceived = 0;
	ackTimeout = 0;	
	
	// ackTimer.reset();
	
	return r;		
}

int fromBufferInt32(unsigned char* data, int start, long *v)
{	
	int i;

	if (currentLength < (start+2)) return 0;
				
	*v = 0;
	for (i=3; i>=0; --i) {
		*v = *v * 256;
		*v += (long) data[start+i];
	}

	return 1;
}

int fromBufferInt32Array(unsigned char* data, long *v, int length)
{
	int i, j;
	
	if ((int) currentLength < length*4) return 0;
	
	for (j=0; j<length; ++j) {
		v[j] = 0;
		for (i=3; i>=0; --i) {
			v[j] = v[j] * 256;
			v[j] += (long) data[j*4+i];
		}
	}

	return 1;
}

int fromBufferVector3Int32(unsigned char* data, unsigned start, long *x, long *y, long *z)
{	
	long v[3];
	int i, j;

	if (currentLength < (start+12)) return 0;
	
	for (i=0; i<3; i++) {
		v[i] = 0;
		for (j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (long) data[i*4+j+start];
		}
	}

	*x = v[0];
	*y = v[1];
	*z = v[2];
	
	return 1;
}

int fromBufferVector3Float(unsigned char* data, unsigned start, float *x, float *y, float *z)
{	
	uint32_t v[3];
	int i, j;

	if (currentLength < (start+12)) return 0;
	
	for (i=0; i<3; i++) {
		v[i] = 0;
		for (j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (long) data[i*4+j+start];
		}
	}

	*x = conItoF(v[0]);
	*y = conItoF(v[1]);
	*z = conItoF(v[2]);
	
	return 1;
}

int fromBufferVector4Float(unsigned char* data, unsigned start, float *q0, float *q1, float *q2, float *q3)
{	
	uint32_t v[4];
	int i, j;

	if (currentLength < (start+16)) return 0;
		
	for (i=0; i<4; i++) {
		v[i] = 0;
		for (j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (long) data[i*4+j+start];
		}
	}

	*q0 = conItoF(v[0]);
	*q1 = conItoF(v[1]);
	*q2 = conItoF(v[2]);
	*q3 = conItoF(v[3]);
	
	return 1;
}

int fromBufferFloat(unsigned char* data, unsigned start, float *v)
{	
	uint32_t i;
	int j;

	if (currentLength < (start+2)) return 0;
		
	i = 0;
	for (j=3; j>=0; --j) {
		i = i * 256;
		i += (long) data[j+start];
	}

	*v = conItoF(i);
	
	return 1;
}

int fromBufferInt16(unsigned char* data, unsigned start, short *v)
{
	int i;

	if (currentLength < (start+2)) return 0;	
	
	*v = 0;
	for (i=1; i>=0; --i) {
		*v = *v * 256;
		*v += (short) data[i+start];
	}

	return 1;
}
