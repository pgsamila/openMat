#include "i2c_mpu.h"

void mpu_i2c_init(void)
{
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	I2C_DeInit(I2C2);
	
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;
	
	I2C_Init(I2C2, &I2C_InitStructure);
	
	I2C_Cmd(I2C2, ENABLE);
}

#define I2C_TIMEOUT 5000
#define I2C_MAX_TRIALS 10

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{	
	uint16_t tR = 0;
	uint32_t tO = 0;
	
	while (tR < I2C_MAX_TRIALS) {
		++tR;

		I2C_GenerateSTART(I2C2, ENABLE);

		tO = 0;	
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
	
		I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Transmitter);	
	
		tO = 0;
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
	
		I2C_SendData(I2C2, reg_addr);	
	
		tO = 0;
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
	
		tO = 0;
		while (length) {
			I2C_SendData(I2C2, *data);
	
			while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && tO < I2C_TIMEOUT) ++tO;
			
			++data;
			--length;
		}
		if (tO >= I2C_TIMEOUT) continue;
	
		I2C_GenerateSTOP(I2C2, ENABLE);

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

		I2C_AcknowledgeConfig(I2C2, ENABLE);
	
		tO = 0;
		while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
	
		I2C_GenerateSTART(I2C2, ENABLE);

		tO = 0;
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
	
		I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Transmitter);

		tO = 0;	
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
		
		I2C_SendData(I2C2, reg_addr);	
	
		tO = 0;
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
	
		I2C_GenerateSTART(I2C2, ENABLE);	
	
		tO = 0;
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;
		
		I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Receiver);
	
		tO = 0;
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && tO < I2C_TIMEOUT) ++tO;
		if (tO >= I2C_TIMEOUT) continue;

		tO = 0;
		while (length && tO < I2C_TIMEOUT) {
			if (length == 1) {
				I2C_AcknowledgeConfig(I2C2, DISABLE);
				I2C_GenerateSTOP(I2C2, ENABLE);
			
				tO = 0;
				while (I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE) == RESET && tO < I2C_TIMEOUT) ++tO;
				if (tO >= I2C_TIMEOUT) continue;
			}
		
			if (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
				*data = I2C_ReceiveData(I2C2);
				
				++data;
				--length;
			}
		}

		return 0;
	}

	return 1;
}