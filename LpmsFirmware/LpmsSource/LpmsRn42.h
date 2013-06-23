/***********************************************************************
** Roving networks Rn42 Bluetooth module control
** 
** Copyright (C) 2013 LP-RESEARCH Inc.
** All rights reserved.
** Contact: info@lp-research.com
***********************************************************************/

#ifndef LPMS_RN42_H
#define LPMS_RN42_H

#include "stm32f2xx.h"
#include "LpmsTimebase.h"
#include "LpmsModbus.h"

// Bluetooth port settings
#define BT_USART_IO_PORT GPIOA
#define BT_CTS_PIN GPIO_Pin_0
#define BT_RTS_PIN GPIO_Pin_1
#define BT_TX_PIN GPIO_Pin_2
#define BT_RX_PIN GPIO_Pin_3
#define BT_CTS_SOURCE GPIO_PinSource0
#define BT_RTS_SOURCE GPIO_PinSource1
#define BT_TX_SOURCE GPIO_PinSource2
#define BT_RX_SOURCE GPIO_PinSource3
#define BT_USART_GPIO_CLK RCC_AHB1Periph_GPIOA
#define BT_USART_CLK RCC_APB1Periph_USART2
#define BT_CTS_AF GPIO_AF_USART2
#define BT_RTS_AF GPIO_AF_USART2
#define BT_TX_AF GPIO_AF_USART2
#define BT_RX_AF GPIO_AF_USART2
#define BT_USART_PORT USART2
#define BT_USART_DR_ADDRESS ((uint32_t)USART2 + 0x04) 
#define BT_USART_DMA_PORT DMA1
#define BT_USART_DMA_CLK RCC_AHB1Periph_DMA1
#define BT_USART_TX_DMA_CHANNEL DMA_Channel_4
#define BT_USART_RX_DMA_CHANNEL DMA_Channel_4
#define BT_USART_TX_DMA_STREAM DMA1_Stream6
#define BT_USART_RX_DMA_STREAM DMA1_Stream5
#define BT_DMA_STEAM_IRQ DMA1_Stream6_IRQn
#define BT_USART_TX_DMA_FLAG_TCIF DMA_FLAG_TCIF6
#define BT_USART_RX_DMA_FLAG_TCIF DMA_FLAG_TCIF5
#define BT_RESET_IO_CLK RCC_AHB1Periph_GPIOC
#define BT_RESET_IO_PORT GPIOC
#define BT_RESET_IO_PIN GPIO_Pin_10
#define BT_PIO6_IO_CLK RCC_AHB1Periph_GPIOC
#define BT_PIO6_IO_PORT GPIOC
#define BT_PIO6_IO_PIN GPIO_Pin_11
#define BT_PIO7_IO_CLK RCC_AHB1Periph_GPIOC
#define BT_PIO7_IO_PORT GPIOC
#define BT_PIO7_IO_PIN GPIO_Pin_12
#define BT_PIO3_IO_CLK RCC_AHB1Periph_GPIOB
#define BT_PIO3_IO_PORT GPIOB
#define BT_PIO3_IO_PIN GPIO_Pin_8
#define BT_PIO4_IO_CLK RCC_AHB1Periph_GPIOB
#define BT_PIO4_IO_PORT GPIOB
#define BT_PIO4_IO_PIN GPIO_Pin_9
#define BT_STATUS_IO_CLK RCC_AHB1Periph_GPIOC
#define BT_STATUS_IO_PORT GPIOC
#define BT_STATUS_IO_PIN GPIO_Pin_3
#define BT_SHOW_STATUS_IO_CLK RCC_AHB1Periph_GPIOC
#define BT_SHOW_STATUS_IO_PORT GPIOC
#define BT_SHOW_STATUS_IO_PIN GPIO_Pin_8

// Bluetooth baudrate enable bits
#define BT_BAUDRATE_9600_ENABLED 0
#define BT_BAUDRATE_19200_ENABLED 1
#define BT_BAUDRATE_38400_ENABLED 2
#define BT_BAUDRATE_57600_ENABLED 3
#define BT_BAUDRATE_115200_ENABLED 4
#define BT_BAUDRATE_230400_ENABLED 5
#define BT_BAUDRATE_460800_ENABLED 6
#define BT_BAUDRATE_921600_ENABLED 7

// Bluetooth baudrate settings
#define BT_BAUDRATE_9600 9600
#define BT_BAUDRATE_19200 19200
#define BT_BAUDRATE_38400 38400
#define BT_BAUDRATE_57600 57600
#define BT_BAUDRATE_115200 115200
#define BT_BAUDRATE_230400 230400
#define BT_BAUDRATE_460800 460800
#define BT_BAUDRATE_921600 921600

// Bluetooth receive buffer length
#define BT_MAX_RX_BUFFER_LENGTH 256

// Bluetooth transmission buffer length
#define BT_MAX_TX_BUFFER_LENGTH 512

// Receive and transmit buffer
extern uint8_t gRxUsartBuffer[BT_MAX_RX_BUFFER_LENGTH];
extern uint8_t gTxUsartBuffer[BT_MAX_TX_BUFFER_LENGTH];

// Sets USART baudrate
void bluetoothSetUSARTConfig(uint32_t baudrate);

// Sets Bluetooth DMA configuration
void bluetoothSetDmaConfig(void);

// Sets Bluetooth GPIO configuration
void bluetoothSetGpioConfig(void);

// Initializes Bluetooth module baudrate
uint8_t bluetoothInitBaudrate(uint32_t baudrateFlag);

// Resets Bluetooth module
void bluetoothReset(void);

// Forces Bluetooth communication baudrate to 9600 KBaud
void bluetoothForceBaudrate9600(void);

// Sets baudrate to firmware default
void bluetoothUseFirmwareBaudrate(void);

// Sets Bluetooth module to master mode
void bluetoothSetMaster(uint8_t isEnabled);

// Sets Bluetooth module command mode
uint8_t bluetoothGotoCommandMode(void);

// Sets Bluetooth module data mode
uint8_t bluetoothGotoDataMode(void);

// Shows Bluetooth status
void bluetoothShowStatus(void);

// Sets Bluetooth module to factory defaults
void bluetoothSetFactoryDefault(uint8_t isEnabled);

// Sets Bluetooth module to autodiscovery mode
void bluetoothSetAutoDiscovery(uint8_t isEnabled);

// Sets Bluetooth module baudrate
uint8_t bluetoothSetBaudrate(uint32_t baudrate);

// Starts Bluetooth data transfer
void bluetoothStartDataTransfer(uint8_t* pDataBuffer, uint16_t dataLength);

// Stops Bluetooth data transfer
void bluetoothStopDataTransfer(void);

// Sets name of Bluetooth adapter
uint8_t bluetoothSetName(void);

// Enables low power connection mode
uint8_t bluetoothEnableLowPowerConnectMode(void);

// Disables low power connection mode
uint8_t bluetoothDisableLowPowerConnectMode(void);

// Enables low latency setting
uint8_t bluetoothEnableLowLatency(void);

// Enables low latency optimization
uint8_t bluetoothEnableLowLatencyOptimization(void);

// Disables special commands 
uint8_t bluetoothDisableSpecialCommands(void);

// chacks if Bluetooth module is ready for sending data
int bluetoothIsReadyForSend(void);

// Checks Bluetooth module connection status
uint8_t bluetoothCheckConnectionStatus(void);

// Polls Bluetooth receive data
uint8_t pollBluetoothData(void);

#endif