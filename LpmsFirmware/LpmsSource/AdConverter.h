#ifndef AD_CONVERTER
#define AD_CONVERTER

#include "stm32f2xx.h"

#define ADC1_DR_ADDRESS ((uint32_t)0x4001204C)

// Scaling with voltage divider 3M / 4.3M
#define VBAT_SCALING 4730

#define N_FORCE_SENSORS 4
#define FORCE_SENSOR_OFFSET 0x7ff

void initAdConverter(void);
float getForceSensorChannel(int c);

#endif