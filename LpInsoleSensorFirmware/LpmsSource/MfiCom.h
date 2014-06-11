#ifndef MFI_COM
#define MFI_COM

#include "stm32f2xx.h"

void mfiI2cInit(void);
int mfiI2cWrite(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int mfiI2cRead(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
void mfiAuthenticate(uint16_t cl, uint8_t* cd);
void sendLingoPacket(uint16_t lingoId,
	uint16_t commandId,
	uint16_t transId,
	uint16_t payloadLength,
	uint8_t* payload);
void parseLingoByte(uint8_t b);
uint8_t pollMfiData(void);

#endif