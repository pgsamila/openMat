#include "Watchdog.h"

volatile uint32_t TimingDelay = 0;
volatile uint32_t LsiFreq = 0;
volatile uint32_t CaptureNumber = 0;
volatile uint32_t PeriodValue = 0;

void initIWatchdog(void)
{
#ifdef ENABLE_WATCHDOG
	// LsiFreq = GetLSIFrequency();
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	IWDG_SetReload(0x0FFF);
	IWDG_ReloadCounter();
	IWDG_Enable();
#endif
}

void resetIWatchdog(void)
{
#ifdef ENABLE_WATCHDOG
	IWDG_ReloadCounter();
#endif    
}

uint32_t GetLSIFrequency(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	RCC_ClocksTypeDef  RCC_ClockFreq;
	
	RCC_LSICmd(ENABLE);
	
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {}
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
	// TIM_RemapConfig(TIM5, TIM5_LSI);	
	TIM_PrescalerConfig(TIM5, 0, TIM_PSCReloadMode_Immediate);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM5, ENABLE);
	
	TIM5->SR = 0;
	
	TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);
		
	while(CaptureNumber != 2) {
	}
	
	TIM_DeInit(TIM5);
	TIM_ITConfig(TIM5, TIM_IT_CC4, DISABLE);
	
	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	if ((RCC->CFGR & RCC_CFGR_PPRE1) == 0) { 
		return ((RCC_ClockFreq.PCLK1_Frequency / PeriodValue) * 8);
	} else {
		return (((2 * RCC_ClockFreq.PCLK1_Frequency) / PeriodValue) * 8) ;
	}
}