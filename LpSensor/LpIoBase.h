/***********************************************************************
** Copyright (c) LP-Research Inc.
** Contact: LP-Research (info@lp-research.com)
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

#ifndef LP_IO_BASE
#define LP_IO_BASE

#ifdef _WIN32
	#include "windows.h"
#endif

#include <iostream>
#include <string>
#include <queue>
#include <fstream>

#include <boost/cstdint.hpp>

#include "LpMatrix.h"
#include "MicroMeasure.h"
#include "LpmsRegisterDefinitions.h"

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

// State machine definitions
#define IDLE_STATE -1
#define ACK_MAX_TIME 3000000
#define MAX_UPLOAD_TIME 20000000
#define MAX_COMMAND_RESEND 3

typedef union _float2uint {
	float fp;
	boost::uint32_t up;
} float2uint;

class LpIoBase {
	public:
		boost::uint32_t conFtoI(float f);
		float conItoF(boost::uint32_t v);

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
		bool fromBufferInt16(unsigned char *data, int *v, int length);
		bool fromBuffer(std::vector<unsigned char> data, float *v, int length);

		bool modbusSetNone(unsigned command);
		bool modbusGet(unsigned command);	
		bool modbusGetMultiUint32(unsigned command, boost::uint32_t *v, int n); 
		bool modbusSetInt32(unsigned command, long v);
		bool modbusSetInt32Array(unsigned command, long *v, int length);
		bool modbusSetVector3Int32(unsigned command, long x, long y, long z);
		bool modbusSetFloat(unsigned command, float v);
		bool modbusSetVector3Float(unsigned command, float x, float y, float z);
		bool modbusSetMatrix3x3f(unsigned command, LpMatrix3x3f m);
		bool modbusSetVector3f(unsigned command, LpVector3f v);
		
		virtual bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
		virtual bool parseModbusByte(void);

		bool isAck(void);
		bool isNack(void);
		bool isWaitForData(void);
		bool isWaitForAck(void);
		int getCurrentState(void);
		void receiveReset(void);
		
		unsigned currentAddress;
		unsigned currentFunction;
		unsigned currentLength;
		
		unsigned lrcIndex;
		unsigned lrcCheck;
		unsigned lrcReceived;

		int currentState;
		
		bool waitForAck;	
		bool ackReceived;
		long ackTimeout;
		
		bool waitForData;
		long dataTimeout;		
		
		MicroMeasure ackTimer;
		bool dataReceived;
		unsigned char cBuffer[512];
		int resendI;
		int cLength;
		long imuId;	
};

#endif