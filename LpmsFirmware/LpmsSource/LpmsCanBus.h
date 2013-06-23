/***********************************************************************
** CAN bus control
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_CANBUS_H
#define LPMS_CANBUS_H

#include "stm32f2xx.h"
#include "LpmsTimebase.h"
#include "LpmsModbus.h"

// Aerospace CAN specific parameters
#define AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_DATATYPE 16
#define AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION 1300

// Used CAN port definitions
#define CAN_PORT 					CAN1
#define CAN_CLK 					RCC_APB1Periph_CAN1
#define CAN_RX_PIN 					GPIO_Pin_11
#define CAN_TX_PIN 					GPIO_Pin_12
#define CAN_GPIO_PORT 				GPIOA
#define CAN_GPIO_CLK 				RCC_AHB1Periph_GPIOA
#define CAN_AF_PORT 				GPIO_AF_CAN1
#define CAN_RX_SOURCE 				GPIO_PinSource11
#define CAN_TX_SOURCE 				GPIO_PinSource12

// Available baudarates
#define CANBUS_BAUDRATE_10K_ENABLED 	0
#define CANBUS_BAUDRATE_20K_ENABLED 	1
#define CANBUS_BAUDRATE_50K_ENABLED 	2
#define CANBUS_BAUDRATE_125K_ENABLED 	3
#define CANBUS_BAUDRATE_250K_ENABLED 	4
#define CANBUS_BAUDRATE_500K_ENABLED 	5
#define CANBUS_BAUDRATE_800K_ENABLED 	6
#define CANBUS_BAUDRATE_1M_ENABLED 		7

// CAN transmission timeout
#define CAN_TX_TIMEOUT 			0xffff

// CANopen definitions
#define MAX_HEARTBEAT_TIME 			0.5f
#define HEARTBEAT_CAN_ID 			0x700
#define CANOPEN_STATE_OPERATIONAL 	0x05

// Initializes CAN baudrate
uint8_t CANInitBaudrate(uint8_t baudrateFlag);

// Sets CAN configuration
void CANConfiguration(uint32_t baudrateFlag);

// Starts CAN data transfer
void CANStartDataTransfer(uint8_t* pDataBuffer, uint16_t dataLength);

// Checks CAN connection status
uint8_t CANCheckConnectionStatus(void);

// Stops CAN data transfer
void CANStopDataTransfer(void);

// Starts AeropspaceCAN transfer
void CANStartStrictCANAerospaceDataTransfer(uint16_t id, uint8_t* pDataBuffer);

// Start CANopen data transfer
void CANStartCANOpenDataTransfer(uint16_t cobId, uint8_t* pDataBuffer);

// Sends CAN custom1 orientation data
uint8_t sendCANCustom1OrientationData(void);

// Sends AerospaceCAN data
uint8_t sendStrictCANAerospaceData(uint8_t function, uint16_t length, uint8_t *data);

// Sends CANopen orientation data
uint8_t sendCANOpenOrientationData(void);

// Checks if CANopen heartbeat should be sent
uint8_t checkCANOpenHeartbeat(void);

// Sends CANopen floating point message
void canSendCanOpenFloatingPoint(void);

// Sends CANopen fixed point message
void canSendCustomFixedPoint(void);

// Sends custom floating point message
void canSendCustomFloatingPoint(void);

// Sends CANopen fixed point message
void canSendCanOpenFixedPoint(void);

#endif
