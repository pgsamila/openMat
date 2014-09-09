/***********************************************************************
** (C) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "stm32f2xx.h"

#include "LpmsRtc.h"

#define RTC_CLOCK_SOURCE_LSI

RTC_InitTypeDef RTC_InitStructure;
RTC_TimeTypeDef RTC_TimeStructure;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef  RTC_TimeStampStructure;
RTC_DateTypeDef  RTC_TimeStampDateStructure;
__IO uint32_t AsynchPrediv = 0, SynchPrediv = 0;

void RTC_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	
	#if defined (RTC_CLOCK_SOURCE_LSI)
		RCC_LSICmd(ENABLE);
		
		while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {
		}
		
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
		
		SynchPrediv = 0xFF;
		AsynchPrediv = 0x7F;
	
	#elif defined (RTC_CLOCK_SOURCE_LSE)
		RCC_LSEConfig(RCC_LSE_ON);
		
		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {
		}
		
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		
		SynchPrediv = 0xFF;
		AsynchPrediv = 0x7F;
	
	#else
		#error Please select the RTC Clock source inside the main.c file
	#endif
	
	RCC_RTCCLKCmd(ENABLE);
	
	RTC_WaitForSynchro();
	
	RTC_TimeStampCmd(RTC_TimeStampEdge_Falling, ENABLE);    
}

void initRtc(void)
{

	if (RTC_ReadBackupRegister(RTC_BKP_DR0) != 0x32F2) {
		RTC_Config();

		RTC_InitStructure.RTC_AsynchPrediv = AsynchPrediv;
		RTC_InitStructure.RTC_SynchPrediv = SynchPrediv;
		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
		
		if (RTC_Init(&RTC_InitStructure) == ERROR) {
		}

    	RTC_TimeStructure.RTC_Hours = 0;
	    RTC_TimeStructure.RTC_Minutes = 0;
	    RTC_TimeStructure.RTC_Seconds = 0;
		RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
	}
}

uint32_t getElapsedTimeInSeconds(void)
{
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	return (uint32_t) RTC_TimeStructure.RTC_Seconds + (uint32_t) RTC_TimeStructure.RTC_Minutes * 60 + 1;
}