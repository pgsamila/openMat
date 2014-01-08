/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsFactorySetting.h"
#ifdef USE_CANBUS_INTERFACE

#include "LpmsRs232Usart.h"

uint8_t 			rs232PortRxBuffer[USART_MAX_RX_BUFFER_LENGTH];
static DMA_InitTypeDef 		rs232PortDMAInitStructure;
static LpmsPacket 		rs232PortNewPacket;
static uint8_t 			rs232PortRxState = PACKET_START;
static uint16_t 		rs232PortRawDataCounter = 0;
static uint16_t 		rs232PortRxDmaBufferPtr = 0;

extern uint8_t connectedInterface;

void rs232PortSetGPIOConfig(void)
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

void rs232PortInit(uint32_t baudrate)
{
	// rs232PortSetGPIOConfig();
	rs232PortSetConfig(baudrate);
	rs232PortSetDmaConfig();
}

void rs232PortSetConfig(uint32_t baudrate)
{
  	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RS232_GPIO_CLK, ENABLE);

	RCC_APB2PeriphClockCmd(RS232_CLK, ENABLE);
	
	// GPIO_PinAFConfig(RS232_IO_PORT, RS232_CTS_SOURCE, RS232_CTS_AF);
	// GPIO_PinAFConfig(RS232_IO_PORT, RS232_RTS_SOURCE, RS232_RTS_AF);
	
	GPIO_PinAFConfig(RS232_IO_PORT, RS232_TX_SOURCE, RS232_TX_AF);
	GPIO_PinAFConfig(RS232_IO_PORT, RS232_RX_SOURCE, RS232_RX_AF);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	/* GPIO_InitStructure.GPIO_Pin = RS232_CTS_PIN;
	GPIO_Init(RS232_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = RS232_RTS_PIN;
	GPIO_Init(RS232_IO_PORT, &GPIO_InitStructure); */
	
	GPIO_InitStructure.GPIO_Pin = RS232_TX_PIN;
	GPIO_Init(RS232_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = RS232_RX_PIN;
	GPIO_Init(RS232_IO_PORT, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // USART_HardwareFlowControl_RTS_CTS;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_DeInit(RS232_PORT);
	USART_Init(RS232_PORT, &USART_InitStructure);
	
	USART_Cmd(RS232_PORT, ENABLE);
}

void rs232PortSetDmaConfig(void)
{
  	RCC_AHB1PeriphClockCmd(RS232_DMA_CLK, ENABLE); 	
	
	DMA_DeInit(RS232_RX_DMA_STREAM);
	DMA_StructInit(&rs232PortDMAInitStructure);
	rs232PortDMAInitStructure.DMA_Channel = RS232_RX_DMA_CHANNEL;
	rs232PortDMAInitStructure.DMA_PeripheralBaseAddr = RS232_DR_ADDRESS;
	rs232PortDMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)rs232PortRxBuffer;
	rs232PortDMAInitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	rs232PortDMAInitStructure.DMA_BufferSize = (uint32_t)RS232_MAX_RX_BUFFER_LENGTH;
	rs232PortDMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	rs232PortDMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	rs232PortDMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	rs232PortDMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	rs232PortDMAInitStructure.DMA_Mode = DMA_Mode_Circular;	
	rs232PortDMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	rs232PortDMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	rs232PortDMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	rs232PortDMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	rs232PortDMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(RS232_RX_DMA_STREAM, &rs232PortDMAInitStructure);

	USART_ClearFlag(RS232_PORT, USART_FLAG_TC);
	
	USART_DMACmd(RS232_PORT, USART_DMAReq_Rx, DISABLE);
	USART_DMACmd(RS232_PORT, USART_DMAReq_Tx, DISABLE);

	DMA_Cmd(RS232_RX_DMA_STREAM, DISABLE);
	DMA_Cmd(RS232_TX_DMA_STREAM, DISABLE);

  	USART_DMACmd(RS232_PORT, USART_DMAReq_Rx, ENABLE);
  	DMA_Cmd(RS232_RX_DMA_STREAM, ENABLE);
}

void rs232PortStartTransfer(uint8_t* pDataBuffer, uint16_t dataLength)
{
	// USART_SendData(RS232_PORT, (uint16_t) 0x49);

	DMA_Cmd(RS232_TX_DMA_STREAM, DISABLE);

	DMA_ClearFlag(RS232_TX_DMA_STREAM, RS232_TX_DMA_FLAG_TCIF);
	USART_ClearFlag(RS232_PORT, USART_FLAG_TC);

	DMA_DeInit(RS232_TX_DMA_STREAM);
	rs232PortDMAInitStructure.DMA_Channel = RS232_TX_DMA_CHANNEL;
	rs232PortDMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)pDataBuffer;
	rs232PortDMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	rs232PortDMAInitStructure.DMA_BufferSize = (uint32_t)dataLength;
	rs232PortDMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(RS232_TX_DMA_STREAM, &rs232PortDMAInitStructure);

	USART_DMACmd(RS232_PORT, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(RS232_TX_DMA_STREAM, ENABLE);
}

void rs232PortStopTransfer(void)
{
	if (DMA_GetFlagStatus(RS232_TX_DMA_STREAM, RS232_TX_DMA_FLAG_TCIF) != RESET) {
		DMA_ClearFlag(RS232_TX_DMA_STREAM, RS232_TX_DMA_FLAG_TCIF);
		DMA_Cmd(RS232_TX_DMA_STREAM, DISABLE);
	}
}

uint8_t rs232PortIsTransferCompleted(void)
{
	if (DMA_GetFlagStatus(RS232_TX_DMA_STREAM, RS232_TX_DMA_FLAG_TCIF) != RESET) return 1;

	return 0;
}

uint8_t rs232PortPollData(void) 
{
	uint8_t b;
 	while(rs232PortRxDmaBufferPtr != (RS232_MAX_RX_BUFFER_LENGTH - 
		DMA_GetCurrDataCounter(RS232_RX_DMA_STREAM))) {
		b = rs232PortRxBuffer[rs232PortRxDmaBufferPtr];
		
		switch (rs232PortRxState) {
		case PACKET_START:
			if (b == 0x3a) {
				rs232PortRxState = PACKET_ADDRESS_LB;
				rs232PortNewPacket.start = b;
				rs232PortRawDataCounter = 0;
			}
			break;
			
		case PACKET_ADDRESS_LB:
			rs232PortNewPacket.address = b;
			rs232PortRxState = PACKET_ADDRESS_HB;
			break;

		case PACKET_ADDRESS_HB:
			rs232PortNewPacket.address = rs232PortNewPacket.address | (((uint16_t)b) << 8);
			rs232PortRxState = PACKET_FUNCTION_LB;
			break;

		case PACKET_FUNCTION_LB:
			rs232PortNewPacket.function = b;
			rs232PortRxState = PACKET_FUNCTION_HB;				
			break;

		case PACKET_FUNCTION_HB:
			rs232PortNewPacket.function = rs232PortNewPacket.function | (((uint16_t)b) << 8);
			rs232PortRxState = PACKET_LENGTH_LB;
			break;
			
		case PACKET_LENGTH_LB:
			rs232PortNewPacket.length = b;
			rs232PortRxState = PACKET_LENGTH_HB;				
			break;

		case PACKET_LENGTH_HB:
			rs232PortNewPacket.length = rs232PortNewPacket.length | (((uint16_t)b) << 8);
			if (rs232PortNewPacket.length != 0) {
				rs232PortRxState = PACKET_RAW_DATA;
			} else {
				rs232PortRxState = PACKET_LRC_CHECK_LB;
			}
			break;
			
		case PACKET_RAW_DATA:
			rs232PortNewPacket.data[rs232PortRawDataCounter] = b;
			rs232PortRawDataCounter++;
			if (rs232PortRawDataCounter == rs232PortNewPacket.length) {
			  	rs232PortRawDataCounter = 0;
				rs232PortRxState = PACKET_LRC_CHECK_LB;
			}
			break;
		
		case PACKET_LRC_CHECK_LB:
			rs232PortNewPacket.lrcCheck = b;
			rs232PortRxState = PACKET_LRC_CHECK_HB;
			break;
			
		case PACKET_LRC_CHECK_HB:
			rs232PortNewPacket.lrcCheck = rs232PortNewPacket.lrcCheck | (((uint16_t)b) << 8);
			rs232PortRxState = PACKET_END_LB;
			break;
		
		case PACKET_END_LB:
			if (b == 0x0d) {
				rs232PortNewPacket.end = b;
				rs232PortRxState = PACKET_END_HB;
			} else {
				rs232PortRxState = PACKET_START;
			}
			break;
		
		case PACKET_END_HB:
			if (b == 0x0a) {
				rs232PortNewPacket.end = rs232PortNewPacket.end | (((uint16_t)b) << 8);
				rs232PortRxState = PACKET_START;
				if (rs232PortNewPacket.lrcCheck == computeCheckSum(rs232PortNewPacket)) {
					addPacketToBuffer(rs232PortNewPacket);
					connectedInterface = RS232_CONNECTED;
				}
			} else {
				rs232PortRxState = PACKET_START;
			}
			break;
		
		default:
			rs232PortRxState = PACKET_START;		
			break;
		}
		
		rs232PortRxDmaBufferPtr++;
		if (rs232PortRxDmaBufferPtr == USART_MAX_RX_BUFFER_LENGTH)
			rs232PortRxDmaBufferPtr = 0;
	}
	
	return 1;
}

#endif