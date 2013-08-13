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
		updateAliveLed();

		if (getCurrentMode() == LPMS_COMMAND_MODE) {
			if (isSensorMeasurementReady() == 1) {
				updateSensorData();
				processSensorData();
			}

#ifdef USE_CANBUS_INTERFACE
			pollSerialPortData();
			pollCanBusData();
#else
			pollBluetoothData();
#endif
			parsePacket();

			sendQueue();
		} else if (getCurrentMode() == LPMS_STREAM_MODE) {
#ifdef USE_CANBUS_INTERFACE
			if (getTimeStep() > LPMS_CU_MEASUREMENT_PERIOD) {
				updateSensorData();
				processSensorData();
			}
#endif

			if (isStreamModeTransferReady() == 1) {
				clearStreamModeTransferReady();

#ifdef USE_BLUETOOTH_INTERFACE
				updateSensorData();
				processSensorData();
#endif

				updateDataTransmission();
			}
			
			sendQueue();

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

			sendQueue();
		}
  	}
}