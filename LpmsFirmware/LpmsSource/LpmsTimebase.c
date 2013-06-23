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
	TIM_TimeBaseStructure.TIM_Prescaler = 300;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(DATA_STREAMING_TIMER, &TIM_TimeBaseStructure);
		
	TIM_SetCounter(DATA_STREAMING_TIMER, 0);
	TIM_Cmd(DATA_STREAMING_TIMER, DISABLE);
	TIM_ClearITPendingBit(DATA_STREAMING_TIMER, TIM_IT_Update);
}

void setSystemStepTimer(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(SYSTEM_STEP_TIMER_CLK, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 300;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(SYSTEM_STEP_TIMER, &TIM_TimeBaseStructure);

	TIM_SetCounter(SYSTEM_STEP_TIMER, 0);
	TIM_Cmd(SYSTEM_STEP_TIMER, DISABLE);
	TIM_ClearITPendingBit(SYSTEM_STEP_TIMER,TIM_IT_Update);
}

void setTimeoutTimer(void)
{
	RCC_APB1PeriphClockCmd(LED_FLASHING_TIMER_CLK, ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 60;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(LED_FLASHING_TIMER, &TIM_TimeBaseStructure);
	TIM_SetCounter(LED_FLASHING_TIMER, 0);
	TIM_Cmd(LED_FLASHING_TIMER, DISABLE);
	TIM_ClearITPendingBit(LED_FLASHING_TIMER, TIM_IT_Update);
}

#pragma optimize=none
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
	TIM_Cmd(SYSTEM_STEP_TIMER, ENABLE);
}

void startTimeoutCounting(void)
{
  	TIM_SetCounter(LED_FLASHING_TIMER, 0);
	TIM_Cmd(LED_FLASHING_TIMER, ENABLE);
}

uint32_t getTimeout(void)
{
	uint32_t counter;
	
	counter = TIM_GetCounter(LED_FLASHING_TIMER);

	return counter;
}

void getTimeStep(float* t)
{
	uint32_t counter;
	counter = TIM_GetCounter(SYSTEM_STEP_TIMER) * 5;

	if (counter == 0) {
		*t = 0.001f;
	} else {
		*t = (float)counter * 0.000001;
	}
}

void setStreamModeTransferReady(void)
{
	if (TIM_GetITStatus(DATA_STREAMING_TIMER, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(DATA_STREAMING_TIMER, TIM_IT_Update);
		isDataStreamReady = 1;   
	}
}

void clearStreamModeTransferReady(void)
{
  	TIM_SetCounter(DATA_STREAMING_TIMER, 0);
	TIM_Cmd(DATA_STREAMING_TIMER, ENABLE);

	isDataStreamReady = 0;
}

uint8_t isStreamModeTransferReady(void)
{
	uint32_t counter;
	
	counter = TIM_GetCounter(DATA_STREAMING_TIMER) * 5;

	if (counter > cycleTime) {
		return 1;
	} else {
		return 0;
	}
}

void startDataStreamTimer(void)
{
	TIM_SetCounter(DATA_STREAMING_TIMER,0);
	TIM_Cmd(DATA_STREAMING_TIMER, ENABLE);
}

void stopDataStreamTimer(void)
{
	TIM_Cmd(DATA_STREAMING_TIMER, DISABLE);
}