/***********************************************************************
** USART control
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_TTL_USART_H
#define LPMS_TTL_USART_H

#include "stm32f2xx.h"
#include "LpmsModbus.h"
#include "LpmsTimebase.h"

// peripheral hardware definition
#define TTL_USART_IO_PORT GPIOC
#define TTL_USART_TX_PIN GPIO_Pin_6
#define TTL_USART_RX_PIN GPIO_Pin_7
#define TTL_USART_TX_SOURCE GPIO_PinSource6
#define TTL_USART_RX_SOURCE GPIO_PinSource7
#define TTL_USART_GPIO_CLK RCC_AHB1Periph_GPIOC
#define TTL_USART_CLK RCC_APB2Periph_USART6
#define TTL_USART_TX_AF GPIO_AF_USART6
#define TTL_USART_RX_AF GPIO_AF_USART6
#define TTL_USART_PORT USART6
#define TTL_USART_DR_ADDRESS ((uint32_t)USART6 + 0x04) 
#define TTL_USART_DMA_PORT DMA2
#define TTL_USART_DMA_CLK RCC_AHB1Periph_DMA2
#define TTL_USART_TX_DMA_CHANNEL DMA_Channel_5
#define TTL_USART_RX_DMA_CHANNEL DMA_Channel_5
#define TTL_USART_TX_DMA_STREAM DMA2_Stream7
#define TTL_USART_RX_DMA_STREAM DMA2_Stream2
#define TTL_USART_DMA_STEAM_IRQ DMA2_Stream7_IRQn
#define TTL_USART_TX_DMA_FLAG_TCIF DMA_FLAG_TCIF7
#define TTL_USART_RX_DMA_FLAG_TCIF DMA_FLAG_TCIF2

// Baudrate definitions
#define TTL_USART_BAUDRATE_9600 9600
#define TTL_USART_BAUDRATE_19200 19200
#define TTL_USART_BAUDRATE_38400 38400
#define TTL_USART_BAUDRATE_57600 57600
#define TTL_USART_BAUDRATE_115200 115200
#define TTL_USART_BAUDRATE_230400 230400
#define TTL_USART_BAUDRATE_460800 460800
#define TTL_USART_BAUDRATE_921600 921600

// USART maximum buffer length definitions
#define TTL_USART_MAX_RX_BUFFER_LENGTH 512

//  initializes serial port
void ttlUsartPortInit(uint32_t baudrate);

// Sets serial port configuration
void ttlUsartPortSetConfig(uint32_t baudrate);

// Sets serial port DMA configuration
void ttlUsartPortSetDmaConfig(void);

// Starts serial port transfer
void ttlUsartPortStartTransfer(uint8_t* pDataBuffer, uint16_t dataLength);

// Stops serial port transfer
void ttlUsartPortStopTransfer(void);

// Polls serial port data
uint8_t ttlUsartPortPollData(void);

uint8_t ttlUsartPortPollDataBle(void);

// Checks if serial port transfer has been completed
uint8_t ttlUsartPortIsTransferCompleted(void);

#endif