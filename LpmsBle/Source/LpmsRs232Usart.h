/***********************************************************************
** USART control
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_RS232USART_H
#define LPMS_RS232USART_H

#include "stm32f37x.h"
#include "LpmsModbus.h"
#include "LpmsTimebase.h"

// peripheral hardware definition
/* #define RS232_IO_PORT GPIOA
#define RS232_TX_PIN GPIO_Pin_9
#define RS232_RX_PIN GPIO_Pin_10
#define RS232_TX_SOURCE GPIO_PinSource9
#define RS232_RX_SOURCE GPIO_PinSource10
#define RS232_GPIO_CLK RCC_AHB1Periph_GPIOA
#define RS232_CLK RCC_APB2Periph_USART1
#define RS232_TX_AF GPIO_AF_USART1
#define RS232_RX_AF GPIO_AF_USART1
#define RS232_PORT USART2
#define RS232_TDR_ADDRESS ((uint32_t)USART2 + 0x28) 
#define RS232_RDR_ADDRESS ((uint32_t)USART2 + 0x24)
#define RS232_DMA_STEAM_IRQ DMA2_Stream7_IRQn
#define RS232_TX_DMA_FLAG_TCIF DMA_FLAG_TCIF7
#define RS232_RX_DMA_FLAG_TCIF DMA_FLAG_TCIF6 */

// Baudrate definitions
#define RS232_BAUDRATE_9600 9600
#define RS232_BAUDRATE_19200 19200
#define RS232_BAUDRATE_38400 38400
#define RS232_BAUDRATE_57600 57600
#define RS232_BAUDRATE_115200 115200
#define RS232_BAUDRATE_230400 230400
#define RS232_BAUDRATE_460800 460800
#define RS232_BAUDRATE_921600 921600

// USART maximum buffer length definitions
#define RS232_MAX_RX_BUFFER_LENGTH 512

//  initializes serial port
void rs232PortInit(uint32_t baudrate);

// Sets serial port configuration
void rs232PortSetConfig(uint32_t baudrate);

// Sets serial port DMA configuration
void rs232PortSetDmaConfig(void);

// Starts serial port transfer
void rs232PortStartTransfer(uint8_t* pDataBuffer, uint16_t dataLength);

// Stops serial port transfer
void rs232PortStopTransfer(void);

// Polls serial port data
uint8_t rs232PortPollData(void);

// Checks if serial port transfer has been completed
uint8_t rs232PortIsTransferCompleted(void);

#endif