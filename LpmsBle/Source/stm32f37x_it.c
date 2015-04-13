/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "stm32f37x_it.h"
#include "main.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}

__IO uint8_t systemStepTimeout = 0;
__IO uint8_t stepDiv = 0;

extern uint8_t lpmsMeasurementIntervals;

void TIM3_IRQHandler(void)
{
	if (TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) != RESET) {
		TIM_ClearFlag(TIM3, TIM_IT_Update);  
		++stepDiv;
		if (stepDiv >= lpmsMeasurementIntervals) {
			systemStepTimeout = 1;
			stepDiv = 0;
		}
	}
}
