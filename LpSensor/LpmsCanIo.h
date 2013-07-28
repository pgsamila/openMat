/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_CAN_IO
#define LPMS_CAN_IO

#include <iostream>
#include <string>
#include <queue>
#include <vector>

#include "ImuData.h"
#include "MicroMeasure.h"
#include "LpmsIoInterface.h"

#ifdef _WIN32
	#include "windows.h"
	#include "PCANBasic.h"
#endif

#include "LpmsRegisterDefinitions.h"

#include <boost/cstdint.hpp>
#include <boost/lexical_cast.hpp>

#define AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_DATATYPE 16 // UCHAR4
#define AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION 1300
#define AEROSPACE_CAN_ROLL 0x138
#define AEROSPACE_CAN_PITCH 0x137
#define AEROSPACE_CAN_YAW 0x141

class LpmsCanIo : public LpmsIoInterface {
public:
	LpmsCanIo(CalibrationData *configData);
	~LpmsCanIo(void);
	bool connect(std::string deviceId);

#ifdef _WIN32
	bool getTxMessage(std::queue<TPCANMsg> *topTxQ);
	bool parseCanMsg(TPCANMsg m);
#endif

protected:
	bool updateSendQueue(void);
	bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
	bool parseModbusByte(void);

	int inMC;
	int prevInMC;
	int outMC;

#ifdef _WIN32
	std::queue<TPCANMsg> txQ;
#endif
};

#endif
