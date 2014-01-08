/***********************************************************************
** LPBUS handling and definitions
**
** Copyright (C) 2013 LP-RESEARCH Inc.
** All rights reserved.
** Contact: info@lp-research.com
***********************************************************************/

#ifndef LPMS_MODBUS
#define LPMS_MODBUS

#include "stm32f2xx.h"

// Maximum packet data
#define MAX_PACKET_DATA_LENGTH 256
#define MAX_RX_PACKET_BUFFER 5

// Connection status LPMS-CU
#define USB_CONNECTED 0
#define CANBUS_CONNECTED 1
#define CANOPEN_CONNECTED 2
#define RS232_CONNECTED 3
#define TTL_UART_CONNECTED 4

// LpBus byte information
#define PACKET_START 0
#define PACKET_ADDRESS_LB 1
#define PACKET_ADDRESS_HB 2
#define PACKET_FUNCTION_LB 3
#define PACKET_FUNCTION_HB 4
#define PACKET_LENGTH_LB 5
#define PACKET_LENGTH_HB 6
#define PACKET_RAW_DATA 7
#define PACKET_LRC_CHECK_LB 8
#define PACKET_LRC_CHECK_HB 9
#define PACKET_END_LB 10
#define PACKET_END_HB 11

// LpBus data packet
typedef struct _LpmsModbusPacket {
	uint8_t start;
	uint16_t address;
	uint16_t function;
	uint16_t length;
	uint8_t data[MAX_PACKET_DATA_LENGTH];
	uint16_t lrcCheck;
	uint16_t end;
} LpmsPacket;

// Calculates check-sum
uint16_t computeCheckSum(LpmsPacket packet);

// Adds one packet to packet buffer
void addPacketToBuffer(LpmsPacket packet);

#endif