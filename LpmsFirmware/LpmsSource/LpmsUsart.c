/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "LpmsFactorySetting.h"
#ifdef USE_CANBUS_INTERFACE

#include "LpmsUsart.h"

uint8_t serialPortRxBuffer[USART_MAX_RX_BUFFER_LENGTH];

static DMA_InitTypeDef serialPortDMAInitStructure;
static LpmsPacket newPacket;
static uint8_t rxState = PACKET_START;
static uint16_t rawDataCounter = 0;
static uint16_t rxDmaBufferPtr = 0;

extern uint8_t connectedInterface;

void serialPortInit(uint32_t baudrate)
{
	serialPortSetConfig(baudrate);
	serialPortSetDmaConfig();
}

void serialPortSetConfig(uint32_t baudrate)
{
  	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(USART_GPIO_CLK, ENABLE);

	RCC_APB1PeriphClockCmd(USART_CLK, ENABLE);
	
	GPIO_PinAFConfig(USART_IO_PORT, USART_CTS_SOURCE, USART_CTS_AF);
	GPIO_PinAFConfig(USART_IO_PORT, USART_RTS_SOURCE, USART_RTS_AF);
	GPIO_PinAFConfig(USART_IO_PORT, USART_TX_SOURCE, USART_TX_AF);
	GPIO_PinAFConfig(USART_IO_PORT, USART_RX_SOURCE, USART_RX_AF);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_InitStructure.GPIO_Pin = USART_CTS_PIN;
	GPIO_Init(USART_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = USART_RTS_PIN;
	GPIO_Init(USART_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = USART_TX_PIN;
	GPIO_Init(USART_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = USART_RX_PIN;
	GPIO_Init(USART_IO_PORT, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_DeInit(USART_PORT);
	USART_Init(USART_PORT, &USART_InitStructure);
	
	USART_Cmd(USART_PORT, ENABLE);
}

void serialPortSetDmaConfig(void)
{
  	RCC_AHB1PeriphClockCmd(USART_DMA_CLK, ENABLE); 	
	
	DMA_DeInit(USART_RX_DMA_STREAM);
	DMA_StructInit(&serialPortDMAInitStructure);
	serialPortDMAInitStructure.DMA_Channel = USART_RX_DMA_CHANNEL;
	serialPortDMAInitStructure.DMA_PeripheralBaseAddr = USART_DR_ADDRESS;
	serialPortDMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)serialPortRxBuffer;
	serialPortDMAInitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	serialPortDMAInitStructure.DMA_BufferSize = (uint32_t)USART_MAX_RX_BUFFER_LENGTH;
	serialPortDMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	serialPortDMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	serialPortDMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	serialPortDMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	serialPortDMAInitStructure.DMA_Mode = DMA_Mode_Circular;	
	serialPortDMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	serialPortDMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	serialPortDMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	serialPortDMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	serialPortDMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART_RX_DMA_STREAM, &serialPortDMAInitStructure);

	USART_ClearFlag(USART_PORT, USART_FLAG_TC);
	
	USART_DMACmd(USART_PORT, USART_DMAReq_Rx, DISABLE);
	USART_DMACmd(USART_PORT, USART_DMAReq_Tx, DISABLE);

	DMA_Cmd(USART_RX_DMA_STREAM, DISABLE);
	DMA_Cmd(USART_TX_DMA_STREAM, DISABLE);

  	USART_DMACmd(USART_PORT, USART_DMAReq_Rx, ENABLE);
  	DMA_Cmd(USART_RX_DMA_STREAM, ENABLE);
}

void serialPortStartTransfer(uint8_t* pDataBuffer, uint16_t dataLength)
{
	DMA_Cmd(USART_TX_DMA_STREAM, DISABLE);

	DMA_ClearFlag(USART_TX_DMA_STREAM, USART_TX_DMA_FLAG_TCIF);
	USART_ClearFlag(USART_PORT, USART_FLAG_TC);

	DMA_DeInit(USART_TX_DMA_STREAM);
	serialPortDMAInitStructure.DMA_Channel = USART_TX_DMA_CHANNEL;
	serialPortDMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)pDataBuffer;
	serialPortDMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	serialPortDMAInitStructure.DMA_BufferSize = (uint32_t)dataLength;
	serialPortDMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(USART_TX_DMA_STREAM, &serialPortDMAInitStructure);

	USART_DMACmd(USART_PORT, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(USART_TX_DMA_STREAM, ENABLE);
}

void serialPortStopTransfer(void)
{
	if (DMA_GetFlagStatus(USART_TX_DMA_STREAM, USART_TX_DMA_FLAG_TCIF) != RESET) {
		DMA_ClearFlag(USART_TX_DMA_STREAM, USART_TX_DMA_FLAG_TCIF);
		DMA_Cmd(USART_TX_DMA_STREAM, DISABLE);
	}
}

uint8_t serialPortIsTransferCompleted(void)
{
	if (DMA_GetFlagStatus(USART_TX_DMA_STREAM, USART_TX_DMA_FLAG_TCIF) != RESET) return 1;

	return 0;
}

uint8_t pollSerialPortData(void) 
{
	uint8_t b;
 	while(rxDmaBufferPtr != (USART_MAX_RX_BUFFER_LENGTH - 
		DMA_GetCurrDataCounter(USART_RX_DMA_STREAM))) {
		b = serialPortRxBuffer[rxDmaBufferPtr];
		
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
				if (newPacket.lrcCheck == computeCheckSum(newPacket)) {
					addPacketToBuffer(newPacket);
					connectedInterface = USB_CONNECTED;
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
		if (rxDmaBufferPtr == USART_MAX_RX_BUFFER_LENGTH)
			rxDmaBufferPtr = 0;
	}
	
	return 1;
}

#endif