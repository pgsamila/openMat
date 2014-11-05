/***********************************************************************
** Timer control
**
** Copyright (C) 2013 LP-RESEARCH Inc.
** All rights reserved.
** Contact: info@lp-research.com
***********************************************************************/

#ifndef __LPMS_TIMEBASE_H
#define __LPMS_TIMEBASE_H

#include "stm32f2xx.h"
#include "SensorManager.h"
#include "LpmsConfig.h"

// Timer configuration definitions
#define DATA_STREAMING_TIMER TIM2
#define DATA_STREAMING_TIMER_CLK RCC_APB1Periph_TIM2
#define DATA_STREAMING_TIMER_INT_CHANNEL TIM2_IRQn
#define DataStreamTimerHandler TIM2_IRQHandler
#define LED_FLASHING_TIMER TIM3
#define LED_FLASHING_TIMER_CLK RCC_APB1Periph_TIM3
#define LED_FLASHING_TIMER_INT_CHANNEL TIM3_IRQn
#define LedFlashTimerHandler TIM3_IRQHandler
#define SYSTEM_STEP_TIMER TIM4
#define SYSTEM_STEP_TIMER_CLK RCC_APB1Periph_TIM4

#define LPMS_LED_PERIOD 400

// I2C bus timeout
#define I2C_TIMEOUT_S 1000

// Initialzes timers
void initTimebase(void);

// Delays code exceution
void msDelay(uint32_t ms);

// Sets data stream timer
void setDataStreamTimer(uint32_t T);

// Sets system step timer
void setSystemStepTimer(void);

// Sets timeout timer
void setTimeoutTimer(void);

// Starts timestep counting
void startTimeStepCounting(void);

// Retrieves current time step value
uint32_t getTimeStep(void);

// Sets ready-for-transfer flag
void setStreamModeTransferReady(void);

// Clears ready-for-streaming-mode-transfer flag
void clearStreamModeTransferReady(void);

// Checks if ready for streaming mode transfer
uint8_t isStreamModeTransferReady(void);

// Starts data streaming timer
void startDataStreamTimer(void);

// Stops data streaming timer
void stopDataStreamTimer(void);

// Starts timeout counting
void startTimeoutCounting(void);

// Retrieves current timeout value
uint32_t getTimeout(void);

void updateAliveLed(void);

uint16_t getTransferCycles(void);

void delay_ms(uint32_t ms);

void get_ms(unsigned long *ms);

#endif