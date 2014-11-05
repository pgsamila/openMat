#ifndef I2C_PMU6050
#define I2C_MPU6050

#include "stm32f2xx.h"

void mpu_i2c_init(void);
int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

#endif