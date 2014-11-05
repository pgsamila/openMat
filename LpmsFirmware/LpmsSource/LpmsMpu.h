#ifndef LPMS_MPU
#define LPMS_MPU

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "LpmsTimebase.h"
#include "i2c_mpu.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "LpmsFactorySetting.h"

void initMpu(void);
void pollMpu(uint32_t t);
void pollDmp(void);
void runSelfTest(void);

#endif