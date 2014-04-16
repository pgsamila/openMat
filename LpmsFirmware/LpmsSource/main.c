/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "stm32f2xx.h"
#include "LpmsStartUp.h"
#include "SensorManager.h"
#include "CommunicationManager.h"
#include "LpmsTimebase.h"

int main(void)
{
	int sendCounter = 0;

	msDelay(100);

	setSystemStepTimer();
	msDelay(100);
	
	setTimeoutTimer();
	msDelay(100);
	
	initSensorManager(); 
	msDelay(100);
	
	initTimebase(); 
	msDelay(100);
	
	initCommunicationManager(); 
	msDelay(100);

#ifdef ENABLE_WATCHDOG
	initWatchdog();
	msDelay(100);
#endif

	while (1) {
		updateAliveLed();

		if (getCurrentMode() == LPMS_COMMAND_MODE) {
		  
			if (getTimeStep() > LPMS_MEASUREMENT_PERIOD) {
				updateSensorData();
				processSensorData();
			}

#ifdef USE_CANBUS_INTERFACE
#ifdef USE_RS232_INTERFACE
			pollSerialPortData();
			rs232PortPollData();
#elif defined USE_TTL_UART_INTERFACE
			pollSerialPortData();			
#ifdef LPMS_BLE
			ttlUsartPortPollDataBle();
#else
			ttlUsartPortPollData();			
#endif
#else
			pollSerialPortData();
			pollCanBusData();
#endif
#else
			pollBluetoothData();
#endif
			parsePacket();

#ifdef LPMS_BLE
			if (isStreamModeTransferReady() == 1) {
				clearStreamModeTransferReady();			  
				sendQueue();
			}
#else		
			sendQueue();
#endif
			
		} else if (getCurrentMode() == LPMS_STREAM_MODE) {
		  
			if (getTimeStep() > (LPMS_MEASUREMENT_PERIOD-50)) {
				while (getTimeStep() < LPMS_MEASUREMENT_PERIOD) {
					asm ("nop");
				}

				updateSensorData();
				processSensorData();

				++sendCounter;
				if (sendCounter >= getTransferCycles()) {
					sendCounter = 0;

					clearStreamModeTransferReady();
					updateDataTransmission();
					
#ifdef LPMS_BLE
					sendQueue();
#endif
				}
			}

#ifndef LPMS_BLE			
			sendQueue();
#endif

#ifdef USE_CANBUS_INTERFACE
#ifdef USE_RS232_INTERFACE
			pollSerialPortData();
			rs232PortPollData();
#elif defined USE_TTL_UART_INTERFACE
			pollSerialPortData();
			
#ifdef LPMS_BLE
			ttlUsartPortPollDataBle();
#else
			ttlUsartPortPollData();			
#endif
			
#else
			pollSerialPortData();
			pollCanBusData();
#endif
#else
			pollBluetoothData();
#endif
			parsePacket();
			
		} else if (getCurrentMode() == LPMS_SLEEP_MODE) {
		  
#ifdef USE_CANBUS_INTERFACE
#ifdef USE_RS232_INTERFACE
			pollSerialPortData();
			rs232PortPollData();
#elif defined USE_TTL_UART_INTERFACE
			pollSerialPortData();
			ttlUsartPortPollData();
#else
			pollSerialPortData();
			pollCanBusData();
#endif
#else
			pollBluetoothData();
#endif
			parsePacket();

			sendQueue();
		}
  	}
}