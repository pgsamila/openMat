/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsFactorySetting.h"
#ifdef USE_BLUETOOTH_INTERFACE

#include "LpmsRn42.h"

uint8_t bluetoothRxBuffer[BT_MAX_RX_BUFFER_LENGTH];
static DMA_InitTypeDef bluetoothDMAInitStructure;
static LpmsPacket newPacket;
static uint8_t rxState = PACKET_START;
static uint16_t rawDataCounter = 0;
static uint16_t rxDmaBufferPtr = 0;

void bluetoothSetUSARTConfig(uint32_t baudrate)
{
  	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(BT_USART_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(BT_USART_CLK, ENABLE);
	
	GPIO_PinAFConfig(BT_USART_IO_PORT, BT_CTS_SOURCE, BT_CTS_AF);
	GPIO_PinAFConfig(BT_USART_IO_PORT, BT_RTS_SOURCE, BT_RTS_AF);
	GPIO_PinAFConfig(BT_USART_IO_PORT, BT_TX_SOURCE, BT_TX_AF);
	GPIO_PinAFConfig(BT_USART_IO_PORT, BT_RX_SOURCE, BT_RX_AF);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_InitStructure.GPIO_Pin = BT_CTS_PIN;
	GPIO_Init(BT_USART_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BT_RTS_PIN;
	GPIO_Init(BT_USART_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BT_TX_PIN;
	GPIO_Init(BT_USART_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BT_RX_PIN;
	GPIO_Init(BT_USART_IO_PORT, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_DeInit(BT_USART_PORT);
	USART_Init(BT_USART_PORT, &USART_InitStructure);
	
	USART_Cmd(BT_USART_PORT, ENABLE);
}

void bluetoothSetDmaConfig(void)
{	
  	RCC_AHB1PeriphClockCmd(BT_USART_DMA_CLK, ENABLE); 	
	
	DMA_DeInit(BT_USART_RX_DMA_STREAM);
	DMA_StructInit(&bluetoothDMAInitStructure);
	bluetoothDMAInitStructure.DMA_Channel = BT_USART_RX_DMA_CHANNEL;
	bluetoothDMAInitStructure.DMA_PeripheralBaseAddr = BT_USART_DR_ADDRESS;
	bluetoothDMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)bluetoothRxBuffer;
	bluetoothDMAInitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	bluetoothDMAInitStructure.DMA_BufferSize = (uint32_t)BT_MAX_RX_BUFFER_LENGTH;
	bluetoothDMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	bluetoothDMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	bluetoothDMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	bluetoothDMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	bluetoothDMAInitStructure.DMA_Mode = DMA_Mode_Circular;	
	bluetoothDMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	bluetoothDMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	bluetoothDMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	bluetoothDMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	bluetoothDMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(BT_USART_RX_DMA_STREAM, &bluetoothDMAInitStructure);

	USART_ClearFlag(BT_USART_PORT, USART_FLAG_TC);
	
	USART_DMACmd(BT_USART_PORT, USART_DMAReq_Rx, DISABLE);
	USART_DMACmd(BT_USART_PORT, USART_DMAReq_Tx, DISABLE);

	DMA_Cmd(BT_USART_RX_DMA_STREAM, DISABLE);
	DMA_Cmd(BT_USART_TX_DMA_STREAM, DISABLE);
}

void bluetoothSetGpioConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(BT_RESET_IO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(BT_PIO6_IO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(BT_PIO7_IO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(BT_PIO3_IO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(BT_PIO4_IO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(BT_STATUS_IO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(BT_SHOW_STATUS_IO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = BT_RESET_IO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BT_RESET_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BT_PIO6_IO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BT_PIO6_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BT_PIO7_IO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BT_PIO7_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BT_PIO3_IO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BT_PIO3_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BT_PIO4_IO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BT_PIO4_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BT_STATUS_IO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BT_STATUS_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BT_SHOW_STATUS_IO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BT_SHOW_STATUS_IO_PORT, &GPIO_InitStructure);
}

void bluetoothStartDataTransfer(uint8_t* pDataBuffer, uint16_t dataLength) 
{
	DMA_Cmd(BT_USART_TX_DMA_STREAM, DISABLE);
	
	DMA_ClearFlag(BT_USART_TX_DMA_STREAM, BT_USART_TX_DMA_FLAG_TCIF);
	USART_ClearFlag(BT_USART_PORT, USART_FLAG_TC);

	DMA_DeInit(BT_USART_TX_DMA_STREAM);
	bluetoothDMAInitStructure.DMA_Channel = BT_USART_TX_DMA_CHANNEL;
	bluetoothDMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)pDataBuffer;
	bluetoothDMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	bluetoothDMAInitStructure.DMA_BufferSize = (uint32_t)dataLength;
	bluetoothDMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(BT_USART_TX_DMA_STREAM, &bluetoothDMAInitStructure);

	USART_DMACmd(BT_USART_PORT, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(BT_USART_TX_DMA_STREAM, ENABLE);
}

void bluetoothStopDataTransfer(void)
{
	if (DMA_GetFlagStatus(BT_USART_TX_DMA_STREAM, BT_USART_TX_DMA_FLAG_TCIF) != RESET) {
		DMA_ClearITPendingBit(BT_USART_TX_DMA_STREAM, BT_USART_TX_DMA_FLAG_TCIF);
	}
}

int bluetoothIsReadyForSend(void)
{
	if (DMA_GetFlagStatus(BT_USART_TX_DMA_STREAM, BT_USART_TX_DMA_FLAG_TCIF) != RESET) return 1;

	return 0;
}

uint8_t checkConnectionStatus(void)
{
	if (GPIO_ReadInputDataBit(BT_STATUS_IO_PORT, BT_STATUS_IO_PIN) == RESET) return 1;

	return 0;
}

uint8_t bluetoothInitBaudrate(uint32_t baudrateFlag)
{
	uint32_t baudrate;
	
	switch (baudrateFlag) {
	case BT_BAUDRATE_9600_ENABLED:
		baudrate = BT_BAUDRATE_9600;
	break;
	
	case BT_BAUDRATE_19200_ENABLED:
		baudrate = BT_BAUDRATE_19200;
	break;
	
	case BT_BAUDRATE_38400_ENABLED:
		baudrate = BT_BAUDRATE_38400;
	break;
	
	case BT_BAUDRATE_57600_ENABLED:
		baudrate = BT_BAUDRATE_57600;
	break;
	
	case BT_BAUDRATE_115200_ENABLED:
		baudrate = BT_BAUDRATE_115200;
	break;
	
	case BT_BAUDRATE_230400_ENABLED:
		baudrate = BT_BAUDRATE_230400;
	break;
	
	case BT_BAUDRATE_460800_ENABLED:
		baudrate = BT_BAUDRATE_460800;
	break;
	
	case BT_BAUDRATE_921600_ENABLED:
		baudrate = BT_BAUDRATE_921600;
	break;
	
	default:
		baudrate = BT_BAUDRATE_230400;
	break;
	}
		
	bluetoothSetGpioConfig();
	bluetoothSetDmaConfig();
	bluetoothSetMaster(0);
	bluetoothSetFactoryDefault(0);
	bluetoothSetAutoDiscovery(0);
	bluetoothSetUSARTConfig(9600);
	bluetoothForceBaudrate9600();
	bluetoothReset();
	bluetoothShowStatus();

	if (!bluetoothGotoCommandMode()) return 0;
	if (!bluetoothSetBaudrate(baudrate)) return 0;

	msDelay(50);
	bluetoothSetName();
	msDelay(50);
	
// #ifdef ENABLE_LOWLATENCY
	bluetoothEnableLowLatency();
	bluetoothEnableLowLatencyOptimization();
/* #else
	bluetoothDisableSpecialCommands();
#endif */
	msDelay(50);
	
	/* bluetoothDisableLowPowerConnectMode();
	msDelay(50); */

	bluetoothUseFirmwareBaudrate();
	bluetoothSetUSARTConfig(baudrate);
	bluetoothReset();
	
  	USART_DMACmd(BT_USART_PORT, USART_DMAReq_Rx, ENABLE);
  	DMA_Cmd(BT_USART_RX_DMA_STREAM, ENABLE);

	return 1;
}

void bluetoothReset(void)
{
	GPIO_WriteBit(BT_RESET_IO_PORT, BT_RESET_IO_PIN, Bit_SET);
	msDelay(10);

	GPIO_WriteBit(BT_RESET_IO_PORT, BT_RESET_IO_PIN, Bit_RESET);
	msDelay(10);

	GPIO_WriteBit(BT_RESET_IO_PORT, BT_RESET_IO_PIN, Bit_SET);
	msDelay(1000);
}

void bluetoothForceBaudrate9600(void)
{
	GPIO_WriteBit(BT_PIO7_IO_PORT, BT_PIO7_IO_PIN, Bit_SET);
}

void bluetoothUseFirmwareBaudrate(void)
{
	GPIO_WriteBit(BT_PIO7_IO_PORT, BT_PIO7_IO_PIN, Bit_RESET);
}

void bluetoothSetMaster(uint8_t isEnabled)
{
	if (isEnabled) {
		GPIO_WriteBit(BT_PIO6_IO_PORT, BT_PIO6_IO_PIN, Bit_SET);
	} else {
		GPIO_WriteBit(BT_PIO6_IO_PORT, BT_PIO6_IO_PIN, Bit_RESET);
	}
}

uint8_t bluetoothGotoCommandMode(void)
{
	uint8_t data[5];
	
	USART_SendData(BT_USART_PORT, '$');
	while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	USART_SendData(BT_USART_PORT, '$');
	while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	USART_SendData(BT_USART_PORT, '$');
	while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	for (uint8_t i = 0; i < 5; i++) {
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_RXNE) == RESET);
		data[i] = USART_ReceiveData(BT_USART_PORT);
	}
	
	if (data[0] == 'C' && data[1] == 'M' && data[2] == 'D' && 
		data[3] == 0x0d && data[4] == 0x0a) {
		return 1;
	} else {
		return 0;
	}
}

uint8_t bluetoothGotoDataMode(void)
{
	uint8_t data[10];
		
	USART_SendData(BT_USART_PORT, 'F');
	while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	USART_SendData(BT_USART_PORT, ',');
	while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	USART_SendData(BT_USART_PORT, '1');
	while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	USART_SendData(BT_USART_PORT, 0x0d);
	while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	for (uint8_t i = 0; i < 5; i++) {
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_RXNE) == RESET);
		data[i] = USART_ReceiveData(BT_USART_PORT);
	}
	
	if (data[0] == 'E' && data[1] == 'N' && data[2] == 'D' && 
		data[3] == 0x0d && data[4] == 0x0a) {
		return 1;
	} else {
		return 0;
	}
}

void bluetoothShowStatus(void)
{
	if (GPIO_ReadInputDataBit(BT_STATUS_IO_PORT, BT_STATUS_IO_PIN) == RESET) {
		GPIO_WriteBit(BT_SHOW_STATUS_IO_PORT, BT_SHOW_STATUS_IO_PIN, Bit_RESET);
	} else {
		GPIO_WriteBit(BT_SHOW_STATUS_IO_PORT, BT_SHOW_STATUS_IO_PIN, Bit_SET);
	}
}

void bluetoothSetFactoryDefault(uint8_t isEnabled)
{
	if (isEnabled) {
		GPIO_WriteBit(BT_PIO4_IO_PORT, BT_PIO4_IO_PIN, Bit_SET);
		msDelay(1);
		
		GPIO_WriteBit(BT_PIO4_IO_PORT, BT_PIO4_IO_PIN, Bit_RESET);
		msDelay(1);
		
		GPIO_WriteBit(BT_PIO4_IO_PORT, BT_PIO4_IO_PIN, Bit_SET);
		msDelay(1);
		
		GPIO_WriteBit(BT_PIO4_IO_PORT, BT_PIO4_IO_PIN, Bit_RESET);
		msDelay(1);
		
		GPIO_WriteBit(BT_PIO4_IO_PORT, BT_PIO4_IO_PIN, Bit_SET);
		msDelay(1);
		
		GPIO_WriteBit(BT_PIO4_IO_PORT, BT_PIO4_IO_PIN, Bit_RESET);
		msDelay(100);
	} else {
		GPIO_WriteBit(BT_PIO4_IO_PORT, BT_PIO4_IO_PIN, Bit_RESET);
	}
}

void bluetoothSetAutoDiscovery(uint8_t isEnabled)
{
	if (isEnabled) {
		GPIO_WriteBit(BT_PIO3_IO_PORT, BT_PIO3_IO_PIN, Bit_SET);
	} else {
		GPIO_WriteBit(BT_PIO3_IO_PORT, BT_PIO3_IO_PIN, Bit_RESET);
	}
}

uint8_t bluetoothSendCommand(uint8_t* command, int length)
{
	uint8_t data[5]; 
  
	for (int i=0; i < length; i++) {
		USART_SendData(BT_USART_PORT, command[i]);
		while (USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);  	
	}
	
	USART_SendData(BT_USART_PORT, 0x0d);
	while (USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	for (int8_t i = 0; i < 5; i++) {
		while (USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_RXNE) == RESET);
		data[i] = USART_ReceiveData(BT_USART_PORT);
	}
	
	if (data[0] == 'A' && data[1] == 'O' && data[2] == 'K' && 
		data[3] == 0x0d && data[4] == 0x0a) {
		return 1;
	} else {
		return 0;
	}	
}	

uint8_t bluetoothSetName(void)
{
	uint8_t c[16] = "SN,LPMS-B";
  
	return bluetoothSendCommand(c, 9);
}

uint8_t bluetoothDisableLowPowerConnectMode(void)
{
	uint8_t c1[16] = "SI,0400";
	uint8_t c2[16] = "SJ,0400";
	
	bluetoothSendCommand(c1, 7);	

	return bluetoothSendCommand(c2, 7);
}

uint8_t bluetoothEnableLowPowerConnectMode(void)
{
	uint8_t c1[16] = "SI,0000";
	uint8_t c2[16] = "SJ,0100";
	
	bluetoothSendCommand(c1, 7);	

	return bluetoothSendCommand(c2, 7);
}

uint8_t bluetoothEnableLowLatencyOptimization(void)
{
	uint8_t c[16] = "ST,0";
	
	return bluetoothSendCommand(c, 4);	
}

uint8_t bluetoothDisableSpecialCommands(void)
{
	uint8_t c[16] = "SQ,0";
	
	return bluetoothSendCommand(c, 4);	
}

uint8_t bluetoothEnableLowLatency(void)
{
	uint8_t c[16] = "SQ,16";
	
	return bluetoothSendCommand(c, 5);	
}

uint8_t bluetoothGotoFastMode(void)
{
	uint8_t c[16] = "F,1";
	
	return bluetoothSendCommand(c, 3);	
}

uint8_t bluetoothGotoSleepMode(void)
{
	uint8_t c[16] = "SW,8320";
	
	return bluetoothSendCommand(c, 7);	
}

uint8_t bluetoothSetBaudrate(uint32_t baudrate)
{
	uint8_t data[5];
	
	USART_SendData(BT_USART_PORT, 'S');
	while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	USART_SendData(BT_USART_PORT, 'U');
	while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	USART_SendData(BT_USART_PORT, ',');
	while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	switch (baudrate) {
	case BT_BAUDRATE_9600:
		USART_SendData(BT_USART_PORT, '9');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
		
		USART_SendData(BT_USART_PORT, '6');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	break;
	
	case BT_BAUDRATE_19200:
		USART_SendData(BT_USART_PORT, '1');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
		
		USART_SendData(BT_USART_PORT, '9');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	break;
	
	case BT_BAUDRATE_38400:
		USART_SendData(BT_USART_PORT, '3');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
		
		USART_SendData(BT_USART_PORT, '8');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);		
	break;
	
	case BT_BAUDRATE_57600:
		USART_SendData(BT_USART_PORT, '5');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
		
		USART_SendData(BT_USART_PORT, '7');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	break;
	
	case BT_BAUDRATE_115200:
		USART_SendData(BT_USART_PORT, '1');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
		
		USART_SendData(BT_USART_PORT, '1');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	break;
	
	case BT_BAUDRATE_230400:
		USART_SendData(BT_USART_PORT, '2');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
		
		USART_SendData(BT_USART_PORT, '3');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	break;
	
	case BT_BAUDRATE_460800:
		USART_SendData(BT_USART_PORT, '4');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
		
		USART_SendData(BT_USART_PORT, '6');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	break;
	
	case BT_BAUDRATE_921600:
		USART_SendData(BT_USART_PORT, '9');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
		
		USART_SendData(BT_USART_PORT, '2');
		while(USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
		
	break;
	
	default:
		return 0;
	break;
	}

	USART_SendData(BT_USART_PORT, 0x0d);
	while (USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_TXE) == RESET);
	
	for (int8_t i = 0; i < 5; i++) {
		while (USART_GetFlagStatus(BT_USART_PORT, USART_FLAG_RXNE) == RESET);
		data[i] = USART_ReceiveData(BT_USART_PORT);
	}
	
	if (data[0] == 'A' && data[1] == 'O' && data[2] == 'K' && 
		data[3] == 0x0d && data[4] == 0x0a) {
		return 1;
	} else {
		return 0;
	}
}

uint8_t pollBluetoothData(void) 
{
	uint8_t b;

 	while(rxDmaBufferPtr != (BT_MAX_RX_BUFFER_LENGTH - DMA_GetCurrDataCounter(BT_USART_RX_DMA_STREAM))) {
		b = bluetoothRxBuffer[rxDmaBufferPtr];
		
		switch (rxState) {
		case PACKET_START:
			if (b == 0x3a) {
				rxState = PACKET_ADDRESS_LB;
				newPacket.start = b;
				rawDataCounter = 0;
			}
			break;
			
		case PACKET_ADDRESS_LB:
			newPacket.address = b;
			rxState = PACKET_ADDRESS_HB;
			break;

		case PACKET_ADDRESS_HB:
			newPacket.address = newPacket.address | (((uint16_t)b) << 8);
			rxState = PACKET_FUNCTION_LB;
			break;

		case PACKET_FUNCTION_LB:
			newPacket.function = b;
			rxState = PACKET_FUNCTION_HB;				
			break;

		case PACKET_FUNCTION_HB:
			newPacket.function = newPacket.function | (((uint16_t)b) << 8);
			rxState = PACKET_LENGTH_LB;
			break;
			
		case PACKET_LENGTH_LB:
			newPacket.length = b;
			rxState = PACKET_LENGTH_HB;				
			break;

		case PACKET_LENGTH_HB:
			newPacket.length = newPacket.length | (((uint16_t)b) << 8);
			if (newPacket.length != 0) {
				rxState = PACKET_RAW_DATA;
			} else {
				rxState = PACKET_LRC_CHECK_LB;
			}
			break;
			
		case PACKET_RAW_DATA:
			newPacket.data[rawDataCounter] = b;
			rawDataCounter++;
			if (rawDataCounter == newPacket.length) {
			  	rawDataCounter = 0;
				rxState = PACKET_LRC_CHECK_LB;
			}
			break;
		
		case PACKET_LRC_CHECK_LB:
			newPacket.lrcCheck = b;
			rxState = PACKET_LRC_CHECK_HB;
			break;
			
		case PACKET_LRC_CHECK_HB:
			newPacket.lrcCheck = newPacket.lrcCheck | (((uint16_t)b) << 8);
			rxState = PACKET_END_LB;
			break;
		
		case PACKET_END_LB:
			if (b == 0x0d) {
				newPacket.end = b;
				rxState = PACKET_END_HB;
			} else {
				rxState = PACKET_START;
			}
			break;
		
		case PACKET_END_HB:
			if (b == 0x0a) {
				newPacket.end = newPacket.end | (((uint16_t)b) << 8);
				rxState = PACKET_START;
				if ((newPacket.lrcCheck == computeCheckSum(newPacket)) /*&& (newPacket.address == getImuID())*/) {
					addPacketToBuffer(newPacket);
				}
			} else {
				rxState = PACKET_START;
			}
			break;
		
		default:
			rxState = PACKET_START;		
			break;
		}
		
		rxDmaBufferPtr++;
		if (rxDmaBufferPtr == BT_MAX_RX_BUFFER_LENGTH)
			rxDmaBufferPtr = 0;
	}

	// bluetoothShowStatus();
	
	return 1;
}

#endif