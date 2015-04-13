/***********************************************************************
** Sensor firmware start-up and initialization
**
** Copyright (C) 2013 LP-RESEARCH Inc.
** All rights reserved.
** Contact: info@lp-research.com
***********************************************************************/

#ifndef LPMS_STARTUP_H
#define LPMS_STARTUP_H
       
#include "stm32f37x.h"
#include "LpmsFactorySetting.h"

// IO port initialization
#define LED_GPIO_CLK RCC_AHB1Periph_GPIOC
#ifdef USE_LPMSCU_NEW
    #define LED_GPIO_PIN GPIO_Pin_5
#else
    #define LED_GPIO_PIN GPIO_Pin_6
#endif

#define LED_GPIO_PORT GPIOC

// Initializes MCU
void initMCU(void);

// Initializes RCC configuration
void setRCCConfig(void);

// Initializes CLK configuration
void setCLKConfig(void);

// Sets general purpose IO port confirguration
void setGPIOConfig(void);

#endif
