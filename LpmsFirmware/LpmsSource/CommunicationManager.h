/***********************************************************************
** Communication manager
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_COMMUNICATION_MANAGER_H
#define LPMS_COMMUNICATION_MANAGER_H

#include "stm32f2xx.h"
#include "SensorManager.h"
#include "LpmsSensorParameters.h"
#include "LpmsConfig.h"
#include "LpmsRn42.h"
#include "LpmsCanBus.h"
#include "LpmsUsart.h"

// Initializes communication manager
void initCommunicationManager(void);

// Checks communication status
uint8_t checkCommunicationStatus(void);

// Returns 1 if sensor is connected to hist
inline uint8_t isConnected(void);

// Sends RS-232 data or CAN bus data depending on configuration
uint8_t sendData(uint16_t address, uint16_t function, uint16_t length, uint8_t *data);

// Checks if new serial port data is available 
uint8_t pollSerialPortData(void);

// Checks if new CAN bus data is available
uint8_t pollCanBusData(void);

// Checks if new Bluetooth data is available
uint8_t pollBluetoothData(void);

// Sends latest data in streaming mode 
void updateDataTransmission(void);

// Resets data data sending flag
void clearDataSendingFlag(void);

// Sends ACK to host
inline void sendAck(void);

// Sends NACK to host
inline void sendNack(void);

// Prepares sending ACK for firmware update
inline void sendFirmwareUpdateAck(void);

// Prepares sending NACK for firmware update
inline void sendFirmwareUpdateNack(void);

// Sends ACK for firmware update
inline void ackForFirmwareUpdate(void);

// Sends NACK for firmware update
inline void nackForFirmwareUpdate(void);

// Parses current data packet
void parsePacket(void);

// Returns 1 if a successful data exchange has taken place
inline uint8_t isDataExchanged(void);

#endif

