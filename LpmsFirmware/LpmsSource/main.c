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
		if (getCurrentMode() == LPMS_COMMAND_MODE) {
#ifdef USE_CANBUS_INTERFACE
			pollSerialPortData();
			pollCanBusData();
#else
			pollBluetoothData();
#endif
			parsePacket();
			updateSensorData();
			processSensorData();

		} else if (getCurrentMode() == LPMS_STREAM_MODE) {
			updateSensorData();
			processSensorData();

			if (isStreamModeTransferReady()) {
				clearStreamModeTransferReady();
				updateDataTransmission();
			}

#ifdef USE_CANBUS_INTERFACE
			pollSerialPortData();
			pollCanBusData();
#else
			pollBluetoothData();
#endif
			parsePacket();

		} else if (getCurrentMode() == LPMS_SLEEP_MODE) {
#ifdef USE_CANBUS_INTERFACE
			pollSerialPortData();
			pollCanBusData();
#else
			pollBluetoothData();
#endif
			parsePacket();
		}
  	}
}