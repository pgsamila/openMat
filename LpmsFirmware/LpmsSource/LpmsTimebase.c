/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "LpmsTimebase.h"

uint8_t isDataStreamReady = 0;
float cycleTime = 10.0f;
uint8_t cyclesPerDataTransfer = 4;
uint16_t ledFlashTime = 400;

uint16_t getTransferCycles(void)
{
	return cyclesPerDataTransfer;
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
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
	
	int clock = 60000000; 		// 60M tick/s -half of system clock of 120Mhz
	int cycle = 800;			// 2.5ms cycle
	int ticks = clock/cycle; 	// 150000
	int prescaler = 15;
	int period = ticks/prescaler;
	TIM_TimeBaseStructure.TIM_Period = period-1; // 2499;
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler-1; // 59;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
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
    Delay(ms);
    /*
	uint32_t iterations;
	
	iterations = ms * 20334;

	for( uint32_t i = 0; i < iterations; i++ ) {
		asm("mov r0, r0");
	}
    */
}

#pragma optimize=none
void Delay(__IO uint32_t nTime)
{
  uint32_t startTick = *((volatile unsigned int *)0xE0001004); 
  while (1)
  {
      uint32_t currentTick= *((volatile unsigned int *)0xE0001004) - startTick;
      float dt = currentTick*1000.0f/SystemCoreClock; // ms
      if (dt > nTime)
        break;
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
	GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, (BitAction)(1-GPIO_ReadOutputDataBit(LED_GPIO_PORT, LED_GPIO_PIN)));
}