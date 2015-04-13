#include "main.h"

#include "SensorManager.h"
#include "CommunicationManager.h"
#include "LpmsTimebase.h"

extern uint16_t ledFlashTime;
extern uint8_t connectedInterface;
extern __IO uint8_t systemStepTimeout;

void usart2SendData(uint8_t* dataBuffer, uint16_t dataLength) 
{
	for (int i=0; i<dataLength; ++i) {
		USART_SendData(USART2, dataBuffer[i]);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	}
}

void uart2receiveData(uint8_t *dataBuffer, uint16_t dataLength)
{
	for (int i=0; i<dataLength; ++i) {
		while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
		dataBuffer[i] = USART_ReceiveData(USART2);
	}
}

int main(void)
{
	int sendCounter = 0;
	uint16_t ledC = 0;
	// float dt = 0.0f;

	// initIWatchdog();

	initAliveLed();
	setSystemStepTimer();

	initSensorManager();
	initCommunicationManager();

	// stopwatch_reset();
	// STOPWATCH_START;

	while (1) {
		if (systemStepTimeout == 1) {
		  
			/* STOPWATCH_STOP;	
			dt = CalcNanosecondsFromStopwatch(m_nStart, m_nStop);
			STOPWATCH_START; */
		  
			systemStepTimeout = 0;
			
			++ledC;
			ledC %= ledFlashTime;

			if (ledC == 0) {
			  updateAliveLed();
			}
			// resetIWatchdog();
			
			//updateSensorData();
			//processSensorData();
			
			++sendCounter;
			if (sendCounter >= getTransferCycles()) {
				sendCounter = 0;

				//if (getCurrentMode() == LPMS_STREAM_MODE) updateDataTransmission();

				//sendQueue();
			}

			//rs232PortPollData();
			//parsePacket();				
		}
  	}
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{ 
	while (1) {
	}
}

#endif

