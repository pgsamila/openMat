/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "stm32f2xx.h"

#include "LpmsStartUp.h"
#include "SensorManager.h"
#include "CommunicationManager.h"
#include "LpmsTimebase.h"
#include "Watchdog.h"
#include "LpStopwatch.h"

extern uint16_t ledFlashTime;
extern uint8_t connectedInterface;
extern __IO uint8_t systemStepTimeout;

int main(void)
{
	int sendCounter = 0;
	uint16_t ledC = 0;
	float dt = 0.0f;

	initIWatchdog();
	setGPIOConfig();
	setSystemStepTimer();	
	initSensorManager();
	initTimebase(); 	
	initCommunicationManager();

#ifdef ENABLE_INSOLE
	initAdConverter();
#endif

	stopwatch_reset();
	// STOPWATCH_START;

	while (1) {
		if (getCurrentMode() == LPMS_COMMAND_MODE) {
			if (systemStepTimeout == 1) {
				systemStepTimeout = 0;

				++ledC;
				ledC %= ledFlashTime;
				if (ledC == 0) updateAliveLed();
				resetIWatchdog();

				updateSensorData();
				processSensorData();

				++sendCounter;
				if (sendCounter >= getTransferCycles()) {
					sendCounter = 0;
					clearStreamModeTransferReady();
					
#ifdef LPMS_BLE
					if (connectedInterface != USB_CONNECTED) {
						sendQueue();
					}
#endif
				}

#ifdef LPMS_BLE			
				if (connectedInterface == USB_CONNECTED) {
					sendQueue();
				}
#else
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
			}			
		} else if (getCurrentMode() == LPMS_STREAM_MODE) {	
			if (systemStepTimeout == 1) {
			  
				STOPWATCH_STOP;	
				dt = CalcNanosecondsFromStopwatch(m_nStart, m_nStop);
				STOPWATCH_START;
			  
				systemStepTimeout = 0;				
				++ledC;
				ledC %= ledFlashTime;
				if (ledC == 0) {
				  updateAliveLed();
				}
				resetIWatchdog();
				
				updateSensorData();
				
				processSensorData();
				
				++sendCounter;
				if (sendCounter >= getTransferCycles()) {
					sendCounter = 0;

					clearStreamModeTransferReady();
					updateDataTransmission();
					
#ifdef LPMS_BLE
					if (connectedInterface != USB_CONNECTED) {
						sendQueue();
					}
#endif
				}


#ifdef LPMS_BLE			
				if (connectedInterface == USB_CONNECTED) sendQueue();
#else
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
				
			}
		}
  	}
}