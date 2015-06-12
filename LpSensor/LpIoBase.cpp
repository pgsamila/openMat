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

#include "LpIoBase.h"

bool LpIoBase::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{	
	return true;
}

bool LpIoBase::parseModbusByte(void)
{
	return true;
}

bool LpIoBase::isAck(void) 
{
	if (currentFunction == REPLY_ACK) {
		return true;
	}
	
	return false;
}

bool LpIoBase::isNack(void) 
{
	if (currentFunction == REPLY_NACK) {
		return true;
	}
	
	return false;
}

bool LpIoBase::isWaitForData(void)
{
	return waitForData;
}

bool LpIoBase::isWaitForAck(void)
{
	return waitForAck;
}

int LpIoBase::getCurrentState(void)
{
	return currentState;
}

void LpIoBase::receiveReset(void) 
{
	currentState = IDLE_STATE;
	
	waitForData = false;
	dataReceived = false;
	waitForAck = false;

	ackTimeout = 0;
	dataTimeout = 0;
	
	resendI = 0;
}

boost::uint32_t LpIoBase::conFtoI(float f)
{
	float2uint f2int;
	f2int.fp = f;
	return f2int.up;
}

float LpIoBase::conItoF(boost::uint32_t v)
{
	float2uint f2int;
	f2int.up = v;
	return f2int.fp;
}

bool LpIoBase::fromBuffer(std::vector<unsigned char> data, long *v)
{	
	if (currentLength < 2) return false;
		
	*v = 0;
	for (int i=3; i>=0; --i) {
		*v = *v * 256;
		*v += (long) data[i];
	}

	return true;
}

bool LpIoBase::fromBufferInt16(unsigned char *data, int *v)
{	
	boost::int16_t i;

	i = (boost::int16_t) data[1] << 8 | data[0];
	*v = i;

	return true;
}

bool LpIoBase::fromBufferInt16(unsigned char *data, int *v, int length)
{	
	boost::int16_t i;

	for (int j=0; j<length; ++j) {
		i = (boost::int16_t) data[j*2+1] << 8 | data[j*2];
		v[j] = i;
	}
	
	return true;
}

bool LpIoBase::fromBuffer(std::vector<unsigned char> data, long *v, int length)
{
	if ((int) currentLength < length*4) return false;
	
	for (int j=0; j<length; ++j) {
		v[j] = 0;
		for (int i=3; i>=0; --i) {
			v[j] = v[j] * 256;
			v[j] += (long) data[j*4+i];
		}
	}

	return true;
}

bool LpIoBase::fromBuffer(std::vector<unsigned char> data, float *v, int length)
{
	boost::uint32_t w;	
	
	if ((int) currentLength < length*4) return false;
	
	for (int j=0; j<length; ++j) {
		w = 0;
		for (int i=3; i>=0; --i) {
			w = w * 256;
			w += (long) data[j*4+i];
		}
		
		v[j] = conItoF(w);
	}

	return true;
}

bool LpIoBase::fromBuffer(std::vector<unsigned char> data, float *v)
{	
	boost::uint32_t w;

	if (currentLength < 2) return false;
		
	w = 0;
	for (int i=3; i>=0; --i) {
		w = w * 256;
		w += (long) data[i];
	}

	*v = conItoF(w);
	
	return true;
}

bool LpIoBase::fromBufferBigEndian(unsigned char *data, float *v)
{	
	boost::uint32_t w;

	if (currentLength < 4) return false;
		
	w = 0;
	for (int i=0; i<4; ++i) {
		w = w * 256;
		w += (long) data[i];
	}

	*v = conItoF(w);
	
	return true;
}

bool LpIoBase::fromBuffer(std::vector<unsigned char> data, long *x, long *y, long *z)
{	
	if (currentLength < 12) return false;
	
	long v[3];
	
	for (int i=0; i<3; i++) {
		v[i] = 0;
		for (int j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (long) data[i*4+j];
		}
	}

	*x = v[0];
	*y = v[1];
	*z = v[2];
	
	return true;
}

bool LpIoBase::fromBuffer(std::vector<unsigned char> data, unsigned start, long *x, long *y, long *z)
{	
	if (currentLength < (start+12)) return false;
	
	long v[3];
	
	for (int i=0; i<3; i++) {
		v[i] = 0;
		for (int j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (long) data[i*4+j+start];
		}
	}

	*x = v[0];
	*y = v[1];
	*z = v[2];
	
	return true;
}

bool LpIoBase::fromBuffer(std::vector<unsigned char> data, float *x, float *y, float *z)
{	
	if (currentLength < 12) return false;
	
	boost::uint32_t v[3];
	
	for (int i=0; i<3; i++) {
		v[i] = 0;
		for (int j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (boost::uint32_t /* long */) data[i*4+j];
		}
	}

	*x = conItoF(v[0]);
	*y = conItoF(v[1]);
	*z = conItoF(v[2]);
	
	return true;
}

bool LpIoBase::fromBuffer(std::vector<unsigned char> data, unsigned start, float *x, float *y, float *z)
{	
	if (currentLength < (start+12)) return false;
	
	boost::uint32_t v[3];
	
	for (int i=0; i<3; i++) {
		v[i] = 0;
		for (int j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (long) data[i*4+j+start];
		}
	}

	*x = conItoF(v[0]);
	*y = conItoF(v[1]);
	*z = conItoF(v[2]);
	
	return true;
}

bool LpIoBase::fromBuffer(std::vector<unsigned char> data, unsigned start, float *q0, float *q1, float *q2, float *q3)
{	
	if (currentLength < (start+16)) return false;
	
	boost::uint32_t v[4];
	
	for (int i=0; i<4; i++) {
		v[i] = 0;
		for (int j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (long) data[i*4+j+start];
		}
	}

	*q0 = conItoF(v[0]);
	*q1 = conItoF(v[1]);
	*q2 = conItoF(v[2]);
	*q3 = conItoF(v[3]);
	
	return true;
}

bool LpIoBase::fromBuffer(std::vector<unsigned char> data, unsigned start, float *v)
{	
	boost::uint32_t i;

	if (currentLength < (2+start)) return false;
		
	i = 0;
	for (int j=3; j>=0; --j) {
		i = i * 256;
		i += (long) data[j+start];
	}

	*v = conItoF(i);
	
	return true;
}

bool LpIoBase::fromBuffer(unsigned char *data, float *v)
{	
	boost::uint32_t i;

	i = 0;
	for (int j=3; j>=0; --j) {
		i = i * 256;
		i += (long) data[j];
	}

	*v = conItoF(i);
	
	return true;
}

bool LpIoBase::fromBufferInt16(std::vector<unsigned char> data, unsigned start, short *v)
{
	*v = 0;
	for (int i=1; i>=0; --i) {
		*v = *v * 256;
		*v += (short) data[i+start];
	}

	return true;
}

bool LpIoBase::modbusSetNone(unsigned command) 
{
	bool r;

	receiveReset();

	cLength = 0;
	r = sendModbusData(imuId, command, 0, cBuffer);
	
	currentState = command;
	waitForAck = true;
	waitForData = false;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;
}

bool LpIoBase::modbusGet(unsigned command) 
{
	bool r;

	receiveReset();

	cLength = 0;
	r = sendModbusData(imuId, command, 0, cBuffer);
	
	currentState = command;
	
	waitForData = true;
	waitForAck = false;	
	dataReceived = false;
	dataTimeout = 0;
	ackTimer.reset();	
	
	return r;
}

bool LpIoBase::modbusGetMultiUint32(unsigned command, boost::uint32_t *v, int n) 
{
	bool r;
	boost::uint32_t t;
	
	receiveReset();

	for (int j=0; j<n; ++j) {
		t = v[j];
		for (int i=0; i<4; ++i) {
			cBuffer[j*4+i] = t & 0xff;
			t = t >> 8;
		}
	}
	
	cLength = n*4;
	r = sendModbusData(imuId, command, n*4, cBuffer);
	
	currentState = command;
	waitForData = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;
}

bool LpIoBase::modbusSetInt32(unsigned command, long v)
{
	bool r;	
	
	receiveReset();

	for (int i=0; i<4; ++i) {
		cBuffer[i] = (unsigned char) (v & 0xff);
		v = v >> 8;
	}
	
	cLength = 4;
	r = sendModbusData(imuId, command, 4, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;
}

bool LpIoBase::modbusSetInt32Array(unsigned command, long *v, int length)
{
	bool r;
	
	receiveReset();

	for (int j=0; j<length; ++j) {
		for (int i=0; i<4; ++i) {
			cBuffer[j*4+i] = (unsigned char) (v[j] & 0xff);
			v[j] = v[j] >> 8;
		}
	}
	
	cLength = 4*length;
	r = sendModbusData(imuId, command, 4*length, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;
}

bool LpIoBase::modbusSetVector3Int32(unsigned command, long x, long y, long z)
{
	bool r;
	long v[3] = { x, y, z };

	receiveReset();
	
	for (int i=0; i<3; ++i) {	
		for (int j=3; j>=0; --j) {
			cBuffer[i*4+j] = (unsigned char) (v[i] & 0xff);
			v[i] = v[i] / 256;
		}
	}
	
	cLength = 12;
	r = sendModbusData(imuId, command, 12, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;
}

bool LpIoBase::modbusSetFloat(unsigned command, float v)
{
	boost::uint32_t i;
	boost::uint32_t m = 0xff;
	bool r;		

	receiveReset();

	i = conFtoI(v);

	for (int j=0; j<4; j++) {
		cBuffer[j] = (unsigned char) (i & 0xff);
		i = i / 256;
	}

	cLength = 4;
	r = sendModbusData(imuId, command, 4, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;		
}

bool LpIoBase::modbusSetVector3Float(unsigned command, float x, float y, float z)
{
	boost::uint32_t i;
	boost::uint32_t m = 0xff;
	float v[3] = { x, y, z };
	bool r;	
	
	receiveReset();

	for (int j=0; j<3; ++j) {	
		i = conFtoI(v[j]);
		for (int k=3; k>=0; --k) {
			cBuffer[j*4+k] = (unsigned char) (i & 0xff);
			i = i / 256;
		}
	}

	cLength = 12;
	r = sendModbusData(imuId, command, 12, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;	
	ackTimer.reset();
	
	return r;		
}

bool LpIoBase::modbusSetVector3f(unsigned command, LpVector3f v)
{
	boost::uint32_t i;	
	bool r;	
	
	receiveReset();

	for (int j=0; j<3; j++) {	
		i = conFtoI(v.data[j]);
		for (int k=0; k<4; k++) {
			cBuffer[j*4+k] = (unsigned char) (i & 0xff);
			i = i / 256;
		}
	}

	cLength = 12;
	r = sendModbusData(imuId, command, 12, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;	
	ackTimer.reset();
	
	return r;		
}

bool LpIoBase::modbusSetMatrix3x3f(unsigned command, LpMatrix3x3f m)
{
	boost::uint32_t i;
	bool r;	
	
	receiveReset();

	for (int j=0; j<3; j++) {	
		for (int l=0; l<3; l++) {
			i = conFtoI(m.data[j][l]);
			for (int k=0; k<4; k++) {
				cBuffer[j*3*4+l*4+k] = (unsigned char) (i & 0xff);
				i = i / 256;
			}
		}
	}

	cLength = 36;
	r = sendModbusData(imuId, command, 36, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;	
	ackTimer.reset();
	
	return r;		
}