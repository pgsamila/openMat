/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "LpmsFactorySetting.h"
#include "LpmsRs232Usart.h"

uint8_t rs232PortRxBuffer[USART_MAX_RX_BUFFER_LENGTH];
static LpmsPacket rs232PortNewPacket;
static uint8_t rs232PortRxState = PACKET_START;
static uint16_t rs232PortRawDataCounter = 0;
static uint16_t rs232PortRxDmaBufferPtr = 0;

extern uint8_t connectedInterface;
extern uint8_t transferFormat;

void rs232PortInit(uint32_t baudrate)
{
	rs232PortSetConfig(baudrate);
	rs232PortSetDmaConfig();
	
  	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
  	DMA_Cmd(DMA1_Channel6, ENABLE);	
}

void rs232PortSetConfig(uint32_t baudrate)
{
  	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);
	
	USART_InitStructure.USART_BaudRate = 9600;

	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_Cmd(USART2, ENABLE);
}

#define RS232_TDR_ADDRESS ((uint32_t)USART2 + 0x28) 
#define RS232_RDR_ADDRESS ((uint32_t)USART2 + 0x24)

void rs232PortSetDmaConfig(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitStructure.DMA_PeripheralBaseAddr = RS232_RDR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&rs232PortRxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = RS232_MAX_RX_BUFFER_LENGTH;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel6, ENABLE);

	DMA_ClearFlag(DMA1_FLAG_TC6);
	USART_DMACmd(USART2, USART_DMAReq_Rx, DISABLE);

	DMA_ClearFlag(DMA1_FLAG_TC7);
	USART_ClearFlag(USART2, USART_FLAG_TC);
}

void rs232PortStartTransfer(uint8_t* pDataBuffer, uint16_t dataLength)
{
	DMA_InitTypeDef DMA_InitStructure;

	DMA_Cmd(DMA1_Channel7, DISABLE);
	USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);
	
	DMA_ClearFlag(DMA1_FLAG_TC7);
	USART_ClearFlag(USART2, USART_FLAG_TC);

	DMA_InitStructure.DMA_PeripheralBaseAddr = RS232_TDR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pDataBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = dataLength;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);

	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Channel7, ENABLE);
}

void rs232PortStopTransfer(void)
{
	if (DMA_GetFlagStatus(DMA1_FLAG_TC7) != RESET) {
		DMA_ClearFlag(DMA1_FLAG_TC7);
		DMA_Cmd(DMA1_Channel7, DISABLE);
	}
}

uint8_t rs232PortIsTransferCompleted(void)
{
	if (DMA_GetFlagStatus(DMA1_FLAG_TC7) != RESET) return 1;

	return 0;
}

uint8_t rs232PortPollData(void) 
{
	uint8_t b;
 	while(rs232PortRxDmaBufferPtr != (RS232_MAX_RX_BUFFER_LENGTH - DMA_GetCurrDataCounter(DMA1_Channel6))) {
		b = rs232PortRxBuffer[rs232PortRxDmaBufferPtr];
		
		switch (rs232PortRxState) {
		case PACKET_START:
			if (b == 0x3a) {
				rs232PortRxState = PACKET_FUNCTION_LB;
				rs232PortNewPacket.start = b;
				rs232PortRawDataCounter = 0;
			}
			break;

		case PACKET_FUNCTION_LB:
			rs232PortNewPacket.function = b;
			rs232PortRxState = PACKET_LENGTH_HB;				
			break;
			
		case PACKET_LENGTH_HB:
			rs232PortNewPacket.length = b;
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
			rs232PortRxState = PACKET_START;
			if (rs232PortNewPacket.lrcCheck == computeCheckSum(rs232PortNewPacket)) {
				addPacketToBuffer(rs232PortNewPacket);
				transferFormat = TRANSFER_FORMAT_LPBUS;
			}			
			break;
				
		default:
			rs232PortRxState = PACKET_START;		
			break;
		}

		rs232PortRxDmaBufferPtr++;
		if (rs232PortRxDmaBufferPtr == USART_MAX_RX_BUFFER_LENGTH) rs232PortRxDmaBufferPtr = 0;
	}
	
	return 1;
}