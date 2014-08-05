#include "stm32f2xx_it.h"

__IO uint8_t systemStepTimeout = 0;
extern __IO uint16_t CCR1_Val;

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
	while (1) {
	}
}

void MemManage_Handler(void)
{
	while (1) {
	}
}

void BusFault_Handler(void)
{
	while (1) {
	}
}

void UsageFault_Handler(void)
{
	while (1) {
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

void DMA1_Stream6_IRQHandler(void)
{
}

void DataStreamTimerHandler(void)
{
}

void CAN1_RX0_IRQHandler(void)
{
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) != RESET) {
		TIM_ClearFlag(TIM3, TIM_IT_Update);  
		systemStepTimeout = 1;
	}
}