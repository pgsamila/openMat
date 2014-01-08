/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsFactorySetting.h"
#ifdef USE_CANBUS_INTERFACE

#include "LpmsTTLUsart.h"

uint8_t 			ttlUsartPortRxBuffer[USART_MAX_RX_BUFFER_LENGTH];
static DMA_InitTypeDef 		ttlUsartPortDMAInitStructure;
static LpmsPacket 		ttlUsartPortNewPacket;
static uint8_t 			ttlUsartPortRxState = PACKET_START;
static uint16_t 		ttlUsartPortRawDataCounter = 0;
static uint16_t 		ttlUsartPortRxDmaBufferPtr = 0;

extern uint8_t connectedInterface;

void ttlUsartPortSetGPIOConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_WriteBit(GPIOA, GPIO_Pin_6, (BitAction)0);
}

void ttlUsartPortInit(uint32_t baudrate)
{
	ttlUsartPortSetConfig(baudrate);
	ttlUsartPortSetDmaConfig();
}

void ttlUsartPortSetConfig(uint32_t baudrate)
{
  	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(TTL_USART_GPIO_CLK, ENABLE);

	RCC_APB2PeriphClockCmd(TTL_USART_CLK, ENABLE);
	
	// GPIO_PinAFConfig(RS232_IO_PORT, RS232_CTS_SOURCE, RS232_CTS_AF);
	// GPIO_PinAFConfig(RS232_IO_PORT, RS232_RTS_SOURCE, RS232_RTS_AF);
	
	GPIO_PinAFConfig(TTL_USART_IO_PORT, TTL_USART_TX_SOURCE, TTL_USART_TX_AF);
	GPIO_PinAFConfig(TTL_USART_IO_PORT, TTL_USART_RX_SOURCE, TTL_USART_RX_AF);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	/* GPIO_InitStructure.GPIO_Pin = RS232_CTS_PIN;
	GPIO_Init(RS232_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = RS232_RTS_PIN;
	GPIO_Init(RS232_IO_PORT, &GPIO_InitStructure); */
	
	GPIO_InitStructure.GPIO_Pin = TTL_USART_TX_PIN;
	GPIO_Init(TTL_USART_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = TTL_USART_RX_PIN;
	GPIO_Init(TTL_USART_IO_PORT, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // USART_HardwareFlowControl_RTS_CTS;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_DeInit(TTL_USART_PORT);
	USART_Init(TTL_USART_PORT, &USART_InitStructure);
	
	USART_Cmd(TTL_USART_PORT, ENABLE);
}

void ttlUsartPortSetDmaConfig(void)
{
  	RCC_AHB1PeriphClockCmd(TTL_USART_DMA_CLK, ENABLE); 	
	
	DMA_DeInit(TTL_USART_RX_DMA_STREAM);
	DMA_StructInit(&ttlUsartPortDMAInitStructure);

	ttlUsartPortDMAInitStructure.DMA_Channel = TTL_USART_RX_DMA_CHANNEL;
	ttlUsartPortDMAInitStructure.DMA_PeripheralBaseAddr = TTL_USART_DR_ADDRESS;
	ttlUsartPortDMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)ttlUsartPortRxBuffer;
	ttlUsartPortDMAInitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	ttlUsartPortDMAInitStructure.DMA_BufferSize = (uint32_t)TTL_USART_MAX_RX_BUFFER_LENGTH;
	ttlUsartPortDMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	ttlUsartPortDMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	ttlUsartPortDMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	ttlUsartPortDMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	ttlUsartPortDMAInitStructure.DMA_Mode = DMA_Mode_Circular;	
	ttlUsartPortDMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	ttlUsartPortDMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	ttlUsartPortDMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	ttlUsartPortDMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	ttlUsartPortDMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(TTL_USART_RX_DMA_STREAM, &ttlUsartPortDMAInitStructure);

	USART_ClearFlag(TTL_USART_PORT, USART_FLAG_TC);
	
	USART_DMACmd(TTL_USART_PORT, USART_DMAReq_Rx, DISABLE);
	USART_DMACmd(TTL_USART_PORT, USART_DMAReq_Tx, DISABLE);

	DMA_Cmd(TTL_USART_RX_DMA_STREAM, DISABLE);
	DMA_Cmd(TTL_USART_TX_DMA_STREAM, DISABLE);

  	USART_DMACmd(TTL_USART_PORT, USART_DMAReq_Rx, ENABLE);
  	DMA_Cmd(TTL_USART_RX_DMA_STREAM, ENABLE);
}

void ttlUsartPortStartTransfer(uint8_t* pDataBuffer, uint16_t dataLength)
{
	// USART_SendData(RS232_PORT, (uint16_t) 0x49);

	DMA_Cmd(TTL_USART_TX_DMA_STREAM, DISABLE);

	DMA_ClearFlag(TTL_USART_TX_DMA_STREAM, TTL_USART_TX_DMA_FLAG_TCIF);
	USART_ClearFlag(TTL_USART_PORT, USART_FLAG_TC);

	DMA_DeInit(TTL_USART_TX_DMA_STREAM);
	ttlUsartPortDMAInitStructure.DMA_Channel = TTL_USART_TX_DMA_CHANNEL;
	ttlUsartPortDMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)pDataBuffer;
	ttlUsartPortDMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	ttlUsartPortDMAInitStructure.DMA_BufferSize = (uint32_t)dataLength;
	ttlUsartPortDMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(TTL_USART_TX_DMA_STREAM, &ttlUsartPortDMAInitStructure);

	USART_DMACmd(TTL_USART_PORT, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(TTL_USART_TX_DMA_STREAM, ENABLE);
}

void ttlUsartPortStopTransfer(void)
{
	if (DMA_GetFlagStatus(TTL_USART_TX_DMA_STREAM, TTL_USART_TX_DMA_FLAG_TCIF) != RESET) {
		DMA_ClearFlag(TTL_USART_TX_DMA_STREAM, TTL_USART_TX_DMA_FLAG_TCIF);
		DMA_Cmd(TTL_USART_TX_DMA_STREAM, DISABLE);
	}
}

uint8_t ttlUsartPortIsTransferCompleted(void)
{
	if (DMA_GetFlagStatus(TTL_USART_TX_DMA_STREAM, TTL_USART_TX_DMA_FLAG_TCIF) != RESET) return 1;

	return 0;
}

uint8_t ttlUsartPortPollData(void) 
{
	uint8_t b;
 	while(ttlUsartPortRxDmaBufferPtr != (TTL_USART_MAX_RX_BUFFER_LENGTH - 
		DMA_GetCurrDataCounter(TTL_USART_RX_DMA_STREAM))) {
		b = ttlUsartPortRxBuffer[ttlUsartPortRxDmaBufferPtr];
		
		switch (ttlUsartPortRxState) {
		case PACKET_START:
			if (b == 0x3a) {
				ttlUsartPortRxState = PACKET_ADDRESS_LB;
				ttlUsartPortNewPacket.start = b;
				ttlUsartPortRawDataCounter = 0;
			}
			break;
			
		case PACKET_ADDRESS_LB:
			ttlUsartPortNewPacket.address = b;
			ttlUsartPortRxState = PACKET_ADDRESS_HB;
			break;

		case PACKET_ADDRESS_HB:
			ttlUsartPortNewPacket.address = ttlUsartPortNewPacket.address | (((uint16_t)b) << 8);
			ttlUsartPortRxState = PACKET_FUNCTION_LB;
			break;

		case PACKET_FUNCTION_LB:
			ttlUsartPortNewPacket.function = b;
			ttlUsartPortRxState = PACKET_FUNCTION_HB;				
			break;

		case PACKET_FUNCTION_HB:
			ttlUsartPortNewPacket.function = ttlUsartPortNewPacket.function | (((uint16_t)b) << 8);
			ttlUsartPortRxState = PACKET_LENGTH_LB;
			break;
			
		case PACKET_LENGTH_LB:
			ttlUsartPortNewPacket.length = b;
			ttlUsartPortRxState = PACKET_LENGTH_HB;				
			break;

		case PACKET_LENGTH_HB:
			ttlUsartPortNewPacket.length = ttlUsartPortNewPacket.length | (((uint16_t)b) << 8);
			if (ttlUsartPortNewPacket.length != 0) {
				ttlUsartPortRxState = PACKET_RAW_DATA;
			} else {
				ttlUsartPortRxState = PACKET_LRC_CHECK_LB;
			}
			break;
			
		case PACKET_RAW_DATA:
			ttlUsartPortNewPacket.data[ttlUsartPortRawDataCounter] = b;
			ttlUsartPortRawDataCounter++;
			if (ttlUsartPortRawDataCounter == ttlUsartPortNewPacket.length) {
			  	ttlUsartPortRawDataCounter = 0;
				ttlUsartPortRxState = PACKET_LRC_CHECK_LB;
			}
			break;
		
		case PACKET_LRC_CHECK_LB:
			ttlUsartPortNewPacket.lrcCheck = b;
			ttlUsartPortRxState = PACKET_LRC_CHECK_HB;
			break;
			
		case PACKET_LRC_CHECK_HB:
			ttlUsartPortNewPacket.lrcCheck = ttlUsartPortNewPacket.lrcCheck | (((uint16_t)b) << 8);
			ttlUsartPortRxState = PACKET_END_LB;
			break;
		
		case PACKET_END_LB:
			if (b == 0x0d) {
				ttlUsartPortNewPacket.end = b;
				ttlUsartPortRxState = PACKET_END_HB;
			} else {
				ttlUsartPortRxState = PACKET_START;
			}
			break;
		
		case PACKET_END_HB:
			if (b == 0x0a) {
				ttlUsartPortNewPacket.end = ttlUsartPortNewPacket.end | (((uint16_t)b) << 8);
				ttlUsartPortRxState = PACKET_START;
				if (ttlUsartPortNewPacket.lrcCheck == computeCheckSum(ttlUsartPortNewPacket)) {
					addPacketToBuffer(ttlUsartPortNewPacket);
					connectedInterface = RS232_CONNECTED;
				}
			} else {
				ttlUsartPortRxState = PACKET_START;
			}
			break;
		
		default:
			ttlUsartPortRxState = PACKET_START;		
			break;
		}
		
		ttlUsartPortRxDmaBufferPtr++;
		if (ttlUsartPortRxDmaBufferPtr == TTL_USART_MAX_RX_BUFFER_LENGTH)
			ttlUsartPortRxDmaBufferPtr = 0;
	}
	
	return 1;
}

#endif