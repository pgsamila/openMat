#ifndef PUSH_WATCHDOG
#define PUSH_WATCHDOG

#include "stm32f37x.h"

void initIWatchdog(void);
uint32_t GetLSIFrequency(void);
void resetIWatchdog(void);

#endif