#include "i2c_mpu.h"

void mpu_i2c_init(void)
{
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	/* RCC_APBPeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APBPeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE); */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_4);

	I2C_DeInit(I2C1);
	
	/* I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000; */

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_Timing = 0xC062121F; // 0x00C0216C;
	
	I2C_Init(I2C1, &I2C_InitStructure);
	
	I2C_Cmd(I2C1, ENABLE);
}

#define I2C_TIMEOUT 5000
#define I2C_MAX_TRIALS 10

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{	
	uint16_t tR = 0;
	uint32_t tO = 0;
	
	while (tR < I2C_MAX_TRIALS) {
		++tR;

		tO = 0;
		while (I2C_GetFlagStatus(I2C1, I2C_ISR_BUSY) != RESET && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
	
		I2C_TransferHandling(I2C1, slave_addr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	
		tO = 0;
		while (I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
	
		I2C_SendData(I2C1, reg_addr);	
	
		tO = 0;
		while (I2C_GetFlagStatus(I2C1, I2C_ISR_TCR) == RESET && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;

		I2C_TransferHandling(I2C1, slave_addr, length, I2C_AutoEnd_Mode, I2C_No_StartStop);

		tO = 0;
		while (length && tO < I2C_TIMEOUT) {
			while (I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET && tO < I2C_TIMEOUT) ++tO;
			if (tO >= I2C_TIMEOUT) continue;

			I2C_SendData(I2C1, *data);
			
			++data;
			--length;
		}
		if (tO >= I2C_TIMEOUT) continue;

		tO = 0;
		while (I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET && tO < I2C_TIMEOUT) ++tO;
	
		I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);

		return 0;
	}
		
	return 1;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	uint16_t tR = 0;
	uint32_t tO = 0;
	
	while (tR < I2C_MAX_TRIALS) {
		++tR;
	
		tO = 0;
		while (I2C_GetFlagStatus(I2C1, I2C_ISR_BUSY) != RESET && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
	
		I2C_TransferHandling(I2C1, slave_addr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

		tO = 0;
		while (I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
	
		I2C_SendData(I2C1, (uint8_t)reg_addr);

		tO = 0;
		while (I2C_GetFlagStatus(I2C1, I2C_ISR_TC) == RESET && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
	
		I2C_TransferHandling(I2C1, slave_addr, length, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

		tO = 0;
		while (length && tO < I2C_TIMEOUT) {
			while (I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET && tO < I2C_TIMEOUT) ++tO;
			if (tO >= I2C_TIMEOUT) continue;

			*data = I2C_ReceiveData(I2C1);

			++data;
			--length;
		}
		if (tO >= I2C_TIMEOUT) continue;

		tO = 0;
		while (I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;

		I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);

		return 0;
	}

	return 1;
}