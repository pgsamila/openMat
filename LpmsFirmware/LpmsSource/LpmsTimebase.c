/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsTimebase.h"

uint8_t isDataStreamReady = 0;
float cycleTime = 10.0f;

void initTimebase(void)
{
	uint32_t config;
	config = getConfigReg();

	if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_5HZ_ENABLED) {
		setDataStreamTimer(LPMS_STREAM_T_5HZ);
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_10HZ_ENABLED) {
		setDataStreamTimer(LPMS_STREAM_T_10HZ);
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_30HZ_ENABLED) {
		setDataStreamTimer(LPMS_STREAM_T_30HZ);
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_50HZ_ENABLED) {
		setDataStreamTimer(LPMS_STREAM_T_50HZ);
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_100HZ_ENABLED) {
		setDataStreamTimer(LPMS_STREAM_T_100HZ);
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_200HZ_ENABLED) {
		setDataStreamTimer(LPMS_STREAM_T_200HZ);
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_500HZ_ENABLED) {
		setDataStreamTimer(LPMS_STREAM_T_500HZ);
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_1000HZ_ENABLED) {
		setDataStreamTimer(LPMS_STREAM_T_1000HZ);
	} else {
		setDataStreamTimer(LPMS_STREAM_FREQ_100HZ);
	}
}

void setDataStreamTimer(uint32_t T)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(DATA_STREAMING_TIMER_CLK, ENABLE);
		
	cycleTime = T;
	
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 6000;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(DATA_STREAMING_TIMER, &TIM_TimeBaseStructure);
		
	TIM_SetCounter(DATA_STREAMING_TIMER, 0);
	TIM_Cmd(DATA_STREAMING_TIMER, ENABLE);
	TIM_ClearITPendingBit(DATA_STREAMING_TIMER, TIM_IT_Update);
}

void setSystemStepTimer(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(SYSTEM_STEP_TIMER_CLK, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 6000;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(SYSTEM_STEP_TIMER, &TIM_TimeBaseStructure);

	TIM_SetCounter(SYSTEM_STEP_TIMER, 0);
	TIM_Cmd(SYSTEM_STEP_TIMER, ENABLE);
	TIM_ClearITPendingBit(SYSTEM_STEP_TIMER,TIM_IT_Update);
}

void setTimeoutTimer(void)
{
	RCC_APB1PeriphClockCmd(LED_FLASHING_TIMER_CLK, ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 6000;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(LED_FLASHING_TIMER, &TIM_TimeBaseStructure);

	TIM_SetCounter(LED_FLASHING_TIMER, 0);
	TIM_Cmd(LED_FLASHING_TIMER, ENABLE);
	TIM_ClearITPendingBit(LED_FLASHING_TIMER, TIM_IT_Update);
}

void msDelay(uint32_t ms)
{
	uint32_t iterations;
	
	iterations = ms * 20334;

	for( uint32_t i = 0; i < iterations; i++ ) {
		asm("mov r0, r0");
	}
}

void startTimeStepCounting(void)
{
  	TIM_SetCounter(SYSTEM_STEP_TIMER, 0);
}

void startTimeoutCounting(void)
{
  	TIM_SetCounter(LED_FLASHING_TIMER, 0);
}

uint32_t getTimeout(void)
{
	uint32_t counter;
	
	counter = TIM_GetCounter(LED_FLASHING_TIMER);

	return counter;
}

uint32_t getTimeStep()
{
	return TIM_GetCounter(SYSTEM_STEP_TIMER);
}

#define SENSOR_MEASUREMENT_T 33

uint8_t isSensorMeasurementReady(void)
{
	if (getTimeStep() > SENSOR_MEASUREMENT_T) return 1;

	return 0;
}

void clearStreamModeTransferReady(void)
{
  	TIM_SetCounter(DATA_STREAMING_TIMER, 0);
}

uint8_t isStreamModeTransferReady(void)
{
	if (TIM_GetCounter(DATA_STREAMING_TIMER) > cycleTime) return 1;
	
	return 0;
}

void updateAliveLed(void)
{
	if (TIM_GetCounter(LED_FLASHING_TIMER) > 5000) {
		GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, (BitAction)(1-GPIO_ReadOutputDataBit(LED_GPIO_PORT, LED_GPIO_PIN)));
		TIM_SetCounter(LED_FLASHING_TIMER, 0);
	}
}