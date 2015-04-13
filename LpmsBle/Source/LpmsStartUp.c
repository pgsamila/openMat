/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "LpmsStartUp.h"

void initMCU(void)
{
  	setRCCConfig();
}

void setRCCConfig(void)
{
  	RCC_DeInit();
	RCC_HSICmd(DISABLE);
	RCC_HSEConfig(RCC_HSE_ON);
	
	if (RCC_WaitForHSEStartUp() == SUCCESS) {
		FLASH_PrefetchBufferCmd(ENABLE);
		FLASH_SetLatency(FLASH_Latency_2);
	  	RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div2);
		RCC_PCLK1Config(RCC_HCLK_Div4);

		uint32_t PLL_M = 25, PLL_N = 240, PLL_P = 2, PLL_Q = 5;
		// RCC_PLLConfig(RCC_PLLCFGR_PLLSRC_HSE, PLL_M, PLL_N, PLL_P, PLL_Q);

		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
		}

		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		while(RCC_GetSYSCLKSource() != 0x08) {
		}
		
		double PLL_VCO = (HSE_VALUE / PLL_M) * PLL_N;
	} else {
		RCC_HSICmd(ENABLE);

		while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET) {
		}

		RCC_HSEConfig(RCC_HSE_OFF);
		RCC_PLLCmd(DISABLE);
		FLASH_PrefetchBufferCmd(ENABLE);
		FLASH_SetLatency(FLASH_Latency_2);
	  	RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div2);
		RCC_PCLK1Config(RCC_HCLK_Div4);

		uint32_t PLL_M = 16, PLL_N = 240, PLL_P = 2, PLL_Q = 5;

		// RCC_PLLConfig(RCC_PLLSource_HSI, PLL_M, PLL_N, PLL_P, PLL_Q);
		RCC_PLLCmd(ENABLE);

		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
		}

		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08) {
		}
		
		double PLL_VCO = (HSI_VALUE / PLL_M) * PLL_N;
	}
	
	SCB->VTOR = FLASH_BASE | 0x00000000;
}

void setCLKConfig(void)
{
}