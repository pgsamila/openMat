/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsFactorySetting.h"
#ifdef USE_CANBUS_INTERFACE

#include "LpmsCanBus.h"

static uint8_t gTransmitMailbox = CAN_NO_MB;
static LpmsPacket newPacket;
static uint8_t rxState = PACKET_START;
static uint16_t rawDataCounter = 0;
uint32_t canBaudrate;

extern LpmsReg gReg;
extern uint8_t connectedInterface;
extern float T;
extern LpmsCalibrationData calibrationData;
extern float canHeartbeatTime;

void CANConfiguration(uint32_t baudrateFlag)
{
	canBaudrate = baudrateFlag;

	CAN_InitTypeDef CAN_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(CAN_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);
	
	GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_RX_SOURCE, CAN_AF_PORT);
	GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_TX_SOURCE, CAN_AF_PORT);
	
	GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN | CAN_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);

	CAN_DeInit(CAN_PORT);
	CAN_StructInit(&CAN_InitStructure);
	
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
	
	switch (baudrateFlag) {
	case CANBUS_BAUDRATE_10K_ENABLED:
		CAN_InitStructure.CAN_Prescaler = 200;
	break;
	
	case CANBUS_BAUDRATE_20K_ENABLED:
		CAN_InitStructure.CAN_Prescaler = 100;
	break;
	
	case CANBUS_BAUDRATE_50K_ENABLED:
		CAN_InitStructure.CAN_Prescaler = 40;
	break;
	
	case CANBUS_BAUDRATE_125K_ENABLED:
		CAN_InitStructure.CAN_Prescaler = 16;
	break;
	
	case CANBUS_BAUDRATE_250K_ENABLED:
		CAN_InitStructure.CAN_Prescaler = 8;
	break;
	
	case CANBUS_BAUDRATE_500K_ENABLED:
		CAN_InitStructure.CAN_Prescaler = 4;
	break;
	
	case CANBUS_BAUDRATE_800K_ENABLED:
		CAN_InitStructure.CAN_Prescaler = 3;
	break;
	
	case CANBUS_BAUDRATE_1M_ENABLED:
		CAN_InitStructure.CAN_Prescaler = 2;
	break;
	
	default:
		CAN_InitStructure.CAN_Prescaler = 2;
	break;
	}
	
	CAN_Init(CAN_PORT, &CAN_InitStructure);

	resetCanId();
}

void resetCanId(void) 
{
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = (((uint32_t) (AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION + getImuID()) << 21) & 0xFFFF0000) >> 16;
	CAN_FilterInitStructure.CAN_FilterIdLow = (CAN_ID_STD | CAN_RTR_DATA) & 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xffe0;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
}

uint8_t CANInitBaudrate(uint8_t baudrateFlag)
{
	CANConfiguration(baudrateFlag);
	
	return 1;
}

void CANStartStrictCANAerospaceDataTransfer(uint16_t id, uint8_t* pDataBuffer)
{
	int i;
	uint32_t timeout;
	CanTxMsg strictMsg;

	strictMsg.StdId = id;
	strictMsg.RTR = CAN_RTR_DATA;
	strictMsg.IDE = CAN_ID_STD;
	strictMsg.DLC = 8;

	for (i=0; i<8; i++) {
		strictMsg.Data[i] = pDataBuffer[i];
	}

	timeout = 0;
	gTransmitMailbox = CAN_NO_MB;
	gTransmitMailbox = CAN_Transmit(CAN_PORT, &strictMsg);

	while ((CAN_TransmitStatus(CAN_PORT, gTransmitMailbox) !=  CANTXOK) && (timeout <= 0xffff)) {
		timeout++;
	}
}

void CANStartDataTransfer(uint8_t* pDataBuffer, uint16_t dataLength)
{
	CanTxMsg txMsg;
	uint32_t timeout = 0;
	txMsg.StdId = AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION + getImuID();
	txMsg.RTR = CAN_RTR_DATA;
	txMsg.IDE = CAN_ID_STD;
	txMsg.DLC = 8;
	
	int p = dataLength / 8;
	int r = dataLength % 8;
	
	txMsg.DLC = 8;
	for (int i=0; i<p; i++) {
		for (int j=0; j<8; j++) {
			txMsg.Data[j] = pDataBuffer[i*8+j];
		}

		gTransmitMailbox = CAN_NO_MB;
		gTransmitMailbox = CAN_Transmit(CAN_PORT, &txMsg);

		timeout = 0;
		while ((CAN_TransmitStatus(CAN_PORT, gTransmitMailbox) != CANTXOK) && (timeout <= CAN_TX_TIMEOUT)) {
			timeout++;
		}

		if (timeout > CAN_TX_TIMEOUT) {
		}
	}	
	
	if (r > 0) {
		txMsg.DLC = r;
		for (int j=0; j<r; j++) {
			txMsg.Data[j] = pDataBuffer[p*8+j];
		}

		gTransmitMailbox = CAN_NO_MB;
		gTransmitMailbox = CAN_Transmit(CAN_PORT, &txMsg);

		timeout = 0;
		while ((CAN_TransmitStatus(CAN_PORT, gTransmitMailbox) != CANTXOK) && (timeout <= CAN_TX_TIMEOUT)) {
			timeout++;
		}

		if (timeout > CAN_TX_TIMEOUT) {
		}
	}
}

uint8_t CANCheckConnectionStatus(void)
{
	return 1;
}

void CANStopDataTransfer(void)
{
}

uint8_t pollCanBusData(void)
{
	CanRxMsg m;
	uint8_t b;
	
	while ((CAN_MessagePending(CAN_PORT, CAN_FIFO0) > 0)) {
		CAN_Receive(CAN_PORT, CAN_FIFO0, &m);

		// if (m.StdId != AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION + getImuID()) continue;

		for (int i=0; i<m.DLC; i++) {
			b = m.Data[i];
		
			switch (rxState) {
			case PACKET_START:
				if (b == 0x3a) {
					rxState = PACKET_ADDRESS_LB;
					newPacket.start = b;
					rawDataCounter = 0;
				}
				break;
				
			case PACKET_ADDRESS_LB:
				newPacket.address = b;
				rxState = PACKET_ADDRESS_HB;
				break;
	
			case PACKET_ADDRESS_HB:
				newPacket.address = newPacket.address | (((uint16_t)b) << 8);
				rxState = PACKET_FUNCTION_LB;
				break;
	
			case PACKET_FUNCTION_LB:
				newPacket.function = b;
				rxState = PACKET_FUNCTION_HB;				
				break;
	
			case PACKET_FUNCTION_HB:
				newPacket.function = newPacket.function | (((uint16_t)b) << 8);
				rxState = PACKET_LENGTH_LB;
				break;
				
			case PACKET_LENGTH_LB:
				newPacket.length = b;
				rxState = PACKET_LENGTH_HB;				
				break;
	
			case PACKET_LENGTH_HB:
				newPacket.length = newPacket.length | (((uint16_t)b) << 8);
				if (newPacket.length != 0) {
					rxState = PACKET_RAW_DATA;
				} else {
					rxState = PACKET_LRC_CHECK_LB;
				}
				break;
				
			case PACKET_RAW_DATA:
				newPacket.data[rawDataCounter] = b;
				rawDataCounter++;
				if (rawDataCounter == newPacket.length) {
					rawDataCounter = 0;
					rxState = PACKET_LRC_CHECK_LB;
				}
				break;
			
			case PACKET_LRC_CHECK_LB:
				newPacket.lrcCheck = b;
				rxState = PACKET_LRC_CHECK_HB;
				break;
				
			case PACKET_LRC_CHECK_HB:
				newPacket.lrcCheck = newPacket.lrcCheck | (((uint16_t)b) << 8);
				rxState = PACKET_END_LB;
				break;
			
			case PACKET_END_LB:
				if (b == 0x0d) {
					newPacket.end = b;
					rxState = PACKET_END_HB;
				} else {
					rxState = PACKET_START;
				}
				break;
			
			case PACKET_END_HB:
				if (b == 0x0a) {
					newPacket.end = newPacket.end | (((uint16_t)b) << 8);
					rxState = PACKET_START;
					if (newPacket.lrcCheck == computeCheckSum(newPacket)/* && 
						newPacket.address == getImuID() */) {
						addPacketToBuffer(newPacket);
						connectedInterface = CANBUS_CONNECTED;
					}
				} else {
					rxState = PACKET_START;
				}
				break;
			
			default:
				rxState = PACKET_START;		
				break;
			}			
		}
	}

	return 1;
}

void CANStartCANOpenDataTransfer(uint16_t cobId, uint8_t* pDataBuffer)
{
	int i;
	uint32_t timeout;
	CanTxMsg canOpenMsg;

	canOpenMsg.StdId = cobId;
	canOpenMsg.RTR = CAN_RTR_DATA;
	canOpenMsg.IDE = CAN_ID_STD;
	canOpenMsg.DLC = 8;

	for (i=0; i<8; i++) canOpenMsg.Data[i] = pDataBuffer[i];

	timeout = 0;
	gTransmitMailbox = CAN_NO_MB;
	gTransmitMailbox = CAN_Transmit(CAN_PORT, &canOpenMsg);

	while ((CAN_TransmitStatus(CAN_PORT, gTransmitMailbox) !=  CANTXOK) && (timeout <= 0xffff)) timeout++;
}

uint8_t checkCANOpenHeartbeat(void)
{
	uint16_t cobId;
	uint8_t data[8];
	int i;

	if (canHeartbeatTime > calibrationData.canHeartbeatTiming) {
		for (i=0; i<8; i++) data[i] = 0;
		cobId = HEARTBEAT_CAN_ID + getImuID();
		data[0] = CANOPEN_STATE_OPERATIONAL;
		CANStartCANOpenDataTransfer(cobId, data);

		canHeartbeatTime = 0.0f;
	}

	return 1;
}

void getCanData(uint8_t* data, int i, int fixed) 
{
	uint32_t v;
	uint16_t dataLength;

	v = calibrationData.canMapping[i];

	if (fixed == 1) {
		switch (v) {
		case 0:
			getGyroXData(data, &dataLength, FLOAT_FIXED_POINT_1000, USE_RADIAN);
		break;
	
		case 1:
			getGyroYData(data, &dataLength, FLOAT_FIXED_POINT_1000, USE_RADIAN);
		break;
	
		case 2:
			getGyroZData(data, &dataLength, FLOAT_FIXED_POINT_1000, USE_RADIAN);
		break;
	
		case 3:
			getRollData(data, &dataLength, FLOAT_FIXED_POINT_1000, USE_RADIAN);
		break;
	
		case 4:
			getPitchData(data, &dataLength, FLOAT_FIXED_POINT_1000, USE_RADIAN);
		break;
	
		case 5:
			getYawData(data, &dataLength, FLOAT_FIXED_POINT_1000, USE_RADIAN);
		break;
	
		case 6:
			getLinAccXData(data, &dataLength, FLOAT_FIXED_POINT_1000);
		break;
	
		case 7:
			getLinAccYData(data, &dataLength, FLOAT_FIXED_POINT_1000);
		break;
	
		case 8:
			getLinAccZData(data, &dataLength, FLOAT_FIXED_POINT_1000);
		break;
	
		case 9:
			getMagXData(data, &dataLength, FLOAT_FIXED_POINT_100);
		break;
	
		case 10:
			getMagYData(data, &dataLength, FLOAT_FIXED_POINT_100);
		break;
	
		case 11:
			getMagZData(data, &dataLength, FLOAT_FIXED_POINT_100);
		break;
	
		case 12:
			getQ0Data(data, &dataLength, FLOAT_FIXED_POINT_1000);
		break;
	
		case 13:
			getQ1Data(data, &dataLength, FLOAT_FIXED_POINT_1000);
		break;
	
		case 14:
			getQ2Data(data, &dataLength, FLOAT_FIXED_POINT_1000);
		break;
	
		case 15:
			getQ3Data(data, &dataLength, FLOAT_FIXED_POINT_1000);
		break;
	
		case 16:
			getAccXData(data, &dataLength, FLOAT_FIXED_POINT_1000);
		break;
	
		case 17:
			getAccYData(data, &dataLength, FLOAT_FIXED_POINT_1000);
		break;
	
		case 18:
			getAccZData(data, &dataLength, FLOAT_FIXED_POINT_1000);
		break;

#ifdef USE_HEAVEMOTION
		case 19:
			getHeaveData(data, &dataLength, FLOAT_FIXED_POINT_1000);
		break;
#endif
		}
	} else {
		switch (v) {
		case 0:
			getGyroXData(data, &dataLength, FLOAT_FULL_PRECISION, USE_RADIAN);
		break;
	
		case 1:
			getGyroYData(data, &dataLength, FLOAT_FULL_PRECISION, USE_RADIAN);
		break;
	
		case 2:
			getGyroZData(data, &dataLength, FLOAT_FULL_PRECISION, USE_RADIAN);
		break;
	
		case 3:
			getRollData(data, &dataLength, FLOAT_FULL_PRECISION, USE_RADIAN);
		break;
	
		case 4:
			getPitchData(data, &dataLength, FLOAT_FULL_PRECISION, USE_RADIAN);
		break;
	
		case 5:
			getYawData(data, &dataLength, FLOAT_FULL_PRECISION, USE_RADIAN);
		break;
	
		case 6:
			getLinAccXData(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 7:
			getLinAccYData(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 8:
			getLinAccZData(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 9:
			getMagXData(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 10:
			getMagYData(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 11:
			getMagZData(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 12:
			getQ0Data(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 13:
			getQ1Data(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 14:
			getQ2Data(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 15:
			getQ3Data(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 16:
			getAccXData(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 17:
			getAccYData(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
	
		case 18:
			getAccZData(data, &dataLength, FLOAT_FULL_PRECISION);
		break;

#ifdef USE_HEAVEMOTION
		case 19:
			getHeaveData(data, &dataLength, FLOAT_FULL_PRECISION);
		break;
#endif
		}
	}
}

void canSendCanOpenFloatingPoint(void)
{
	int i, j, k, l;
	uint8_t data[8];
	uint16_t id;	

	l = 0;

	for (j=0; j<4; ++j) {
		for (i=0; i<8; ++i) data[i] = 0;

		for (k=0; k<2; ++k) {
			getCanData(&(data[k*4]), l, 0);
			++l;
		}

		id = 0x180 + (j * 0x100) + getImuID();
		CANStartCANOpenDataTransfer(id, data);
	}
}

void canSendCanOpenFixedPoint(void)
{
	int i, j, k, l;
	uint8_t data[8];
	uint16_t id;	

	l = 0;

	for (j=0; j<4; ++j) {
		for (i=0; i<8; ++i) data[i] = 0;

		for (k=0; k<4; ++k) {
			getCanData(&(data[k*2]), l, 1);
			++l;
		}
		
		id = 0x180 + (j * 0x100) + getImuID();
		CANStartCANOpenDataTransfer(id, data);
	}
}

void canSendCustomFixedPoint(void)
{
	uint8_t i, j, k, l;
	uint16_t startId;
	uint8_t data[8];
	uint16_t id;

	l = 0;

	startId = (gReg.data[LPMS_CAN_CONFIGURATION] & 0xffff0000) >> 16;

	for (j=0; j<4; ++j) {
		for (i=0; i<8; ++i) data[i] = 0;

		for (k=0; k<4; ++k) {
			getCanData(&(data[k*2]), l, 1);
			++l;
		}

		id = (uint16_t) startId + ((getImuID()-1) * 8) + j;
		CANStartCANOpenDataTransfer(id, data);
	}
}

void canSendCustomFloatingPoint(void)
{
	uint8_t i, j, k, l;
	uint32_t startId;
	uint16_t id;
	uint8_t data[8];
	
	l = 0;

	startId = (gReg.data[LPMS_CAN_CONFIGURATION] & 0xffff0000) >> 16;

	for (j=0; j<4; ++j) {
		for (i=0; i<8; ++i) data[i] = 0;

		for (k=0; k<2; ++k) {
			getCanData(&(data[k*4]), l, 0);
			++l;
		}

		id = (uint16_t) startId + ((getImuID()-1) * 8) + j;
		CANStartCANOpenDataTransfer(id, data);
	}
}

uint8_t sendCANOpenOrientationData(void)
{
	uint8_t v = gReg.data[LPMS_CAN_CONFIGURATION];

	if ((v & LPMS_CAN_SEQUENTIAL_MODE) != 0) {
		if ((gReg.data[LPMS_CAN_CONFIGURATION] & LPMS_CAN_FIXEDPOINT_MODE) != 0) {
			canSendCustomFixedPoint();
		} else {
			canSendCustomFloatingPoint();
		}
	} else {
		if ((gReg.data[LPMS_CAN_CONFIGURATION] & LPMS_CAN_FIXEDPOINT_MODE) != 0) {
			canSendCanOpenFixedPoint();
		} else {
			canSendCanOpenFloatingPoint();		
		}	
		checkCANOpenHeartbeat();
	}

	return 1;  
}

#endif