/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsCanIo.h"

LpmsCanIo::LpmsCanIo(CalibrationData *configData) :
	LpmsIoInterface(configData)
{
}

LpmsCanIo::~LpmsCanIo(void)
{
	close();
}

bool LpmsCanIo::connect(std::string deviceId) 
{
	currentState = IDLE_STATE;
	
	waitForAck = false;
	waitForData = false;
	ackReceived = false;

	ackTimeout = 0;
	dataReceived = false;
	dataTimeout = 0;

	lpmsStatus = 0;
	configReg = 0;

	prevInMC = 255;
	outMC = 0;

	rxState = PACKET_END;
	configSet = false;
	
	timestampOffset = 0.0f;
	currentTimestamp = 0.0f;

	setCommandMode();			

	try {
		imuId = boost::lexical_cast<int>(deviceId);
	} catch(boost::bad_lexical_cast const&) {
		imuId = 1;
	}	
	
	return true;
}

bool LpmsCanIo::sendModbusData(unsigned address, unsigned function, 
	unsigned length, unsigned char *data)	
{
	TPCANMsg sendMsg;
	unsigned char txData[1024];
	unsigned int txLrcCheck;
	
	if (length > 1014) return false;

	txData[0] = 0x3a;
	txData[1] = address & 0xff;
	txData[2] = (address >> 8) & 0xff;
	txData[3] = function & 0xff;
	txData[4] = (function >> 8) & 0xff;
	txData[5] = length & 0xff;
	txData[6] = (length >> 8) & 0xff;
	
	for (unsigned int i=0; i < length; ++i) {
		txData[7+i] = data[i];
	}
	
	txLrcCheck = address;
	txLrcCheck += function;
	txLrcCheck += length;
	
	for (unsigned int i=0; i < length; i++) {
		txLrcCheck += data[i];
	}
	
	txData[7 + length] = txLrcCheck & 0xff;
	txData[8 + length] = (txLrcCheck >> 8) & 0xff;
	txData[9 + length] = 0x0d;
	txData[10 + length] = 0x0a;
	
	sendMsg.ID = AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION;
	sendMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;
	
	int p = (length+11) / 8;
	int r = (length+11) % 8;
	
	// printf("Msg. n=%d r=%d f=%d\n", p, r, function);
	
	sendMsg.LEN = 8;
	for (int i=0; i<p; i++) {
		for (int j=0; j<8; j++) {
			sendMsg.DATA[j] = txData[i*8+j];
		}
		/* printf("Msg. push %x %x %x %x %x %x %x %x\n", sendMsg.DATA[0], sendMsg.DATA[1], sendMsg.DATA[2], sendMsg.DATA[3], sendMsg.DATA[4], sendMsg.DATA[5], sendMsg.DATA[6], sendMsg.DATA[7]); */
		txQ.push(sendMsg);
	}
	
	if (r > 0) {
		sendMsg.LEN = r;
		for (int j=0; j<r; j++) {
			sendMsg.DATA[j] = txData[p*8+j];
		}
		/* printf("Msg. push %x %x %x %x %x %x %x %x\n", sendMsg.DATA[0], sendMsg.DATA[1], sendMsg.DATA[2], sendMsg.DATA[3], sendMsg.DATA[4], sendMsg.DATA[5], sendMsg.DATA[6], sendMsg.DATA[7]); */
		txQ.push(sendMsg);
	}	
	
	return true;
}

/* bool LpmsCanIo::sendModbusData(unsigned address, unsigned function, 
	unsigned length, unsigned char *data)	
{
	boost::uint8_t txData[1024];
	TPCANMsg sendMsg;

	memset(txData, 0, 1024);

	if (length > 1024) return false;
			
	txData[0] = 0x3a;
	txData[1] = (boost::uint8_t) ((boost::uint16_t) length & 0xff);
	txData[2] = (boost::uint8_t) (((boost::uint16_t) length & 0xff00) >> 8) & 0xff;
	txData[3] = 0;
	
	for (unsigned i=0; i < length; ++i) {
		txData[4+i] = (boost::uint8_t) data[i];
	}
	
	// txData[4 + length] = 0x0d;
	// txData[5 + length] = 0x0a;
		
	sendMsg.ID = AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION;
	sendMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;

	sendMsg.DATA[0] = (boost::uint8_t) ((boost::uint16_t) address) & 0xff;
	sendMsg.DATA[1] = (boost::uint8_t) AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_DATATYPE;
	sendMsg.DATA[2] = (boost::uint8_t) ((boost::uint16_t) function) & 0xff;

	for (unsigned i=0; i<length+5; i+=4) {
		sendMsg.DATA[3] = (boost::uint8_t) outMC;
		sendMsg.DATA[4] = txData[i];
		sendMsg.DATA[5] = txData[i+1];
		sendMsg.DATA[6] = txData[i+2];
		sendMsg.DATA[7] = txData[i+3];

		if (outMC < 255) ++outMC; else outMC = 0;

		txQ.push(sendMsg);
	}
	
	int p = length / 8;
	int r = length % 8;
	
	sendMsg.LEN = 8;
	for (int i=0; i<length/8; i++) {
		for (int j=0; j<8; j++) {
			sendMsg.DATA[j] = txData[i*8+j];
		}
		txQ.push(sendMsg);
	}	
	
	if (r > 0) {
		sendMsg.LEN = r;
		for (int j=0; j<r; j++) {
			sendMsg.DATA[j] = txData[p*8+j];
		}
	}
		
	return true;
} */

bool LpmsCanIo::getTxMessage(std::queue<TPCANMsg> *topTxQ)
{
	int i = txQ.size();
	if (i <= 0) return false;

	topTxQ->push(txQ.front());
	txQ.pop();

	return true;
}
	
bool LpmsCanIo::parseCanMsg(TPCANMsg m)
{
	const float r2d = 57.2958f;
	
	int l = m.LEN;

	/* if ((configReg & LPMS_STREAM_CAN_CUSTOM1) != 0 &&
		(lpmsStatus & LPMS_STREAM_MODE) != 0) {
		fromBufferBigEndian(&(m.DATA[4]), &v);
	
		switch (m.ID) {
		case AEROSPACE_CAN_ROLL:
			imuData.r[0] = v * r2d;
		break;
		
		case AEROSPACE_CAN_PITCH:
			imuData.r[1] = v * r2d;
		break;

		case AEROSPACE_CAN_YAW:
			imuData.r[2] = v * r2d;	
		break;
		}
		
		receiveReset();		
		
		return true;
	} else if ((configReg & LPMS_STREAM_CAN_OPEN) != 0 &&
		(lpmsStatus & LPMS_STREAM_MODE) != 0) {		
		if (m.ID == (0x180 + imuId)) {
			fromBuffer(&(m.DATA[0]), &v);
			imuData.r[0] = v * r2d;
			fromBuffer(&(m.DATA[4]), &v);
			imuData.r[1] = v * r2d;
		}	
		if (m.ID == (0x280 + imuId)) {
			fromBuffer(&(m.DATA[0]), &v);
			imuData.r[2] = v * r2d;
		}
		
		receiveReset();	

		return true;
	} else { */
		for (int i=0; i<m.LEN; i++) {
			dataQueue.push((unsigned char) m.DATA[i]);
		}
		
	// printf("Msg. push %x %x %x %x %x %x %x %x\n", m.DATA[0], m.DATA[1], m.DATA[2], m.DATA[3], m.DATA[4], m.DATA[5], m.DATA[6], m.DATA[7]);	
		
		return true;
	// }
}

bool LpmsCanIo::parseModbusByte(unsigned char b)
{	
	switch (rxState) {
	case PACKET_END:
		if (b == 0x3a) {
			rxState = PACKET_ADDRESS0;
			oneTx.clear();
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
		rxState = PACKET_RAW_DATA;
		rawDataIndex = currentLength;
	break;
			
	case PACKET_RAW_DATA:
		if (rawDataIndex == 0) {
			lrcCheck = currentAddress + currentFunction + currentLength;
			for (unsigned i=0; i<oneTx.size(); i++) {
				lrcCheck += oneTx[i];
			}
			
			lrcReceived = b;
			rxState = PACKET_LRC_CHECK1;			
		} else {	
			oneTx.push_back(b);		
			--rawDataIndex;		
		}
		break;
		
	case PACKET_LRC_CHECK1:
		lrcReceived = lrcReceived + ((unsigned) b * 256);
		
		if (lrcReceived == lrcCheck) {
			parseFunction();
			// cout << "[LPMS-U] Finished processing packet: " << c << endl;
			// c++;
		} else {
			cout << "[LpmsCanIo] Checksum fail in data packet" << endl;
		}
		
		rxState = PACKET_END;
		break;
	
	default:
		rxState = PACKET_END;		
		return false;
		break;
	}
	
	return true;
}

/* bool LpmsCanIo::parseModbusByte(unsigned char b)
{	
	switch (rxState) {
	case PACKET_END:
		if (b == 0x3a) rxState = PACKET_LENGTH0;
	break;

	case PACKET_LENGTH0:
		currentLength = (unsigned) b;
		rxState = PACKET_LENGTH1;				
	break;

	case PACKET_LENGTH1:
		currentLength = currentLength + b * 256;
		rawDataIndex = currentLength;
	
		oneTx.clear();
		
		rxState = PACKET_SKIP_ZERO;
	break;

	case PACKET_SKIP_ZERO:
		rxState = PACKET_RAW_DATA;
	break;
	
	case PACKET_RAW_DATA:		
		if (rawDataIndex == 0) {
			parseFunction();
			rxState = PACKET_END;				
		} else {		
			oneTx.push_back(b);
			--rawDataIndex;					
		}
		break;
	
	default:
		rxState = PACKET_END;
		return false;
		break;
	}
	
	return true;
} */