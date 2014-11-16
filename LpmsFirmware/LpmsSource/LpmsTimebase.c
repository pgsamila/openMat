/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "LpmsTimebase.h"

uint8_t isDataStreamReady = 0;
float cycleTime = 10.0f;
uint8_t cyclesPerDataTransfer = 4;
uint16_t ledFlashTime = LPMS_LED_PERIOD;

void initTimebase(void)
{
	uint32_t config;
	config = getConfigReg();

	if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_5HZ_ENABLED) {
		cyclesPerDataTransfer = 64; // 5.75 Hz
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_10HZ_ENABLED) {
		cyclesPerDataTransfer = 80;	// 10Hz			//32; // 12.5 Hz
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_30HZ_ENABLED) {
		cyclesPerDataTransfer = 32;	// 25Hz			//16; // 25 Hz
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_50HZ_ENABLED) {
		cyclesPerDataTransfer = 16; // 50Hz 		//8; // 50 Hz
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_100HZ_ENABLED) {
		cyclesPerDataTransfer = 2;  // 400Hz 		//4; // 100 Hz
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_200HZ_ENABLED) {
#ifdef USE_BLUETOOTH_INTERFACE
		cyclesPerDataTransfer = 1;  // 800Hz   		//3; // 133 Hz
#else
		cyclesPerDataTransfer = 2; // 200 Hz
#endif
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_500HZ_ENABLED) {
		cyclesPerDataTransfer = 1; // 400 Hz
	} else if ((config & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_1000HZ_ENABLED) {
		cyclesPerDataTransfer = 1; // 400 Hz
	} else {
		cyclesPerDataTransfer = 4; // 100 Hz
	}
}

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
	TIM_TimeBaseStructure.TIM_Period = period-1;//2499;
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler-1; //59;
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