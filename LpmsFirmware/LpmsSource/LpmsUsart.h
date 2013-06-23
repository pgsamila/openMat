/***********************************************************************
** USART control
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_USART_H
#define LPMS_USART_H

#include "stm32f2xx.h"
#include "LpmsModbus.h"
#include "LpmsTimebase.h"

// peripheral hardware definition
#define USART_IO_PORT GPIOA
#define USART_CTS_PIN GPIO_Pin_0
#define USART_RTS_PIN GPIO_Pin_1
#define USART_TX_PIN GPIO_Pin_2
#define USART_RX_PIN GPIO_Pin_3
#define USART_CTS_SOURCE GPIO_PinSource0
#define USART_RTS_SOURCE GPIO_PinSource1
#define USART_TX_SOURCE GPIO_PinSource2
#define USART_RX_SOURCE GPIO_PinSource3
#define USART_GPIO_CLK RCC_AHB1Periph_GPIOA
#define USART_CLK RCC_APB1Periph_USART2
#define USART_CTS_AF GPIO_AF_USART2
#define USART_RTS_AF GPIO_AF_USART2
#define USART_TX_AF GPIO_AF_USART2
#define USART_RX_AF GPIO_AF_USART2
#define USART_PORT USART2
#define USART_DR_ADDRESS ((uint32_t)USART2 + 0x04) 
#define USART_DMA_PORT DMA1
#define USART_DMA_CLK RCC_AHB1Periph_DMA1
#define USART_TX_DMA_CHANNEL DMA_Channel_4
#define USART_RX_DMA_CHANNEL DMA_Channel_4
#define USART_TX_DMA_STREAM DMA1_Stream6
#define USART_RX_DMA_STREAM DMA1_Stream5
#define USART_DMA_STEAM_IRQ DMA1_Stream6_IRQn
#define USART_TX_DMA_FLAG_TCIF DMA_FLAG_TCIF6
#define USART_RX_DMA_FLAG_TCIF DMA_FLAG_TCIF5

// Baudrate definitions
#define USART_BAUDRATE_9600 9600
#define USART_BAUDRATE_19200 19200
#define USART_BAUDRATE_38400 38400
#define USART_BAUDRATE_57600 57600
#define USART_BAUDRATE_115200 115200
#define USART_BAUDRATE_230400 230400
#define USART_BAUDRATE_460800 460800
#define USART_BAUDRATE_921600 921600

// USART maximum buffer length definitions
#define USART_MAX_RX_BUFFER_LENGTH 256
#define USART_MAX_TX_BUFFER_LENGTH 512

//  initializes serial port
void serialPortInit(uint32_t baudrate);

// Sets serial port configuration
void serialPortSetConfig(uint32_t baudrate);

// Sets serial port DMA configuration
void serialPortSetDmaConfig(void);

// Starts serial port transfer
void serialPortStartTransfer(uint8_t* pDataBuffer, uint16_t dataLength);

// Stops serial port transfer
void serialPortStopTransfer(void);

// Polls serial port data
uint8_t pollSerialPortData(void);

// Checks if serial port transfer has been completed
uint8_t serialPortIsTransferCompleted(void);

#endif