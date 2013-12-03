/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/ 

#include "stm32f2xx.h"

#define USE_LPMSCU_NEW

#ifdef USE_LPMSCU_NEW
    #define LED_GPIO_PIN GPIO_Pin_5
#else
    #define LED_GPIO_PIN GPIO_Pin_6
#endif

#define USER_APPLICATION_ADDRESS	(uint32_t)0x08000000
#define USER_APPLICATION_BACKUP_ADDRESS	(uint32_t)0x08020000

// maximum user flash size is 128Kbyte
#define MAX_USER_FLASH_SIZE	126
#define USER_FLASH_END_SECTOR	FLASH_Sector_4

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000)
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000)
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000)
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000)
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000)
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000)
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000)
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000)
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000)
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000)
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000)
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000)

typedef  void (*pFunction)(void);

__IO uint32_t gIsFlashProtection = 0;
uint8_t gIsFlashSuccess = 0;
uint32_t JumpAddress;
pFunction Jump_To_Application;
uint8_t buf_1024[1024];

void msDelay(uint32_t ms)
{
	uint32_t iterations;
	
	iterations = ms * 20334;
	for( uint32_t i = 0; i < iterations; i++ ) {
		asm("mov r0, r0");
	}
}

void initFlash(void)
{ 
	FLASH_Unlock(); 

	// Clear pending flags (if any)
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
}

static uint32_t getSector(uint32_t Address)
{
	uint32_t sector = 0;
	
	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0)) {
		sector = FLASH_Sector_0;
	} else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1)) {
		sector = FLASH_Sector_1;
	} else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2)) {
		sector = FLASH_Sector_2;
	} else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3)) {
		sector = FLASH_Sector_3;
	} else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4)) {
		sector = FLASH_Sector_4;
	} else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5)) {
		sector = FLASH_Sector_5;
	} else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6)) {
		sector = FLASH_Sector_6;
	} else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7)) {
		sector = FLASH_Sector_7;
	} else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8)) {
		sector = FLASH_Sector_8;
	} else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9)) {
		sector = FLASH_Sector_9;
	} else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10)) {
		sector = FLASH_Sector_10;
	} else {
		sector = FLASH_Sector_11;
	}

	return sector;
}

uint16_t getFlashWriteProtectionStatus(void)
{
	uint32_t user_start_sector;
	
	// Get the sector where start the user flash area
	user_start_sector = getSector(USER_APPLICATION_ADDRESS);
	
	// Check if there are write protected sectors inside the user flash area
	if ((FLASH_OB_GetWRP() >> (user_start_sector/8)) == (0xFFF >> (user_start_sector/8))) {
		// No write protected sectors inside the user flash area
		return 1;
	} else { 
		// Some sectors inside the user flash area are write protected
		return 0;
	}
}

uint32_t disableFlashWriteProtection(void)
{
	__IO uint32_t user_start_sector, user_wrp_sectors;
	
	// Get the sector where start the user flash area
	user_start_sector = getSector(USER_APPLICATION_ADDRESS);
	
	// Mark all sectors inside the user flash area as non protected
	user_wrp_sectors = 0xFFF-((1 << (user_start_sector/8))-1);
	
	// Unlock the Option Bytes
	FLASH_OB_Unlock();
	
	// Disable the write protection for all sectors inside the user flash area
	FLASH_OB_WRPConfig(user_wrp_sectors, DISABLE);
	
	// Start the Option Bytes programming process
	if (FLASH_OB_Launch() != FLASH_COMPLETE) {
		// Error: Flash write unprotection failed
		return 0;
	}
	
	// Write Protection successfully disabled
	return 1;
}

void copyFlashToRam_1K(__IO uint32_t* flashAddress, uint32_t* ramBuffer)
{
	uint16_t data_length;
	
	// 1K flash copied to ram
	data_length = 1024/4;
	
	for (uint16_t i = 0; i < data_length; i++ ) {
		*(uint32_t*)(ramBuffer + i) = *(uint32_t*)*flashAddress;
		*flashAddress += 4;
	}
}

uint8_t copyRamToFlash_1K(__IO uint32_t* flashAddress, uint32_t* ramBuffer)
{
	uint16_t data_length;
	
	// 1K ram copied to flash
	data_length = 1024/4;
	
	for (uint16_t i = 0; i < data_length; i++) {
		if (FLASH_ProgramWord(*flashAddress, *(uint32_t*)(ramBuffer + i)) == FLASH_COMPLETE) {
			// Check the written value
			if (*(uint32_t*)*flashAddress != *(uint32_t*)(ramBuffer + i)) {
				// Flash content doesn't match SRAM content
				return 0;
			}

			// Increment FLASH destination address
			*flashAddress += 4;
		} else {
			// Error occurred while writing data in Flash memory
			return 0;
		}
	}
	
	return 1;
}

uint8_t eraseFlash(uint32_t startSector)
{
	uint32_t user_start_sector;
	
	// Get the sector where start the user flash area
	user_start_sector = getSector(USER_APPLICATION_ADDRESS);
	
	for(uint32_t i = user_start_sector; i <= USER_FLASH_END_SECTOR; i += 8)
	{
		if (FLASH_EraseSector(i, VoltageRange_3) != FLASH_COMPLETE)
		{
			// Error occurred while page erase
			return 0;
		}
	}
	
	return 1;
}

int main(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// Configure Output Led 1 Status
	GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	for (uint16_t i = 0; i < 1024; i++) {
		buf_1024[i] = 1;
	}
	
  	initFlash();
	if (getFlashWriteProtectionStatus() == 0) {
		gIsFlashProtection = 1;
	} else {
		gIsFlashProtection = 0;
	}
	
	if (gIsFlashProtection == 1) {
		if (disableFlashWriteProtection()) {
			gIsFlashProtection = 0;
		}
	}
	
	if (gIsFlashProtection == 0) {
		uint32_t flash_source, ram, flash_destination;
		
		flash_source = USER_APPLICATION_BACKUP_ADDRESS;
		flash_destination = USER_APPLICATION_ADDRESS;
		ram = (uint32_t)buf_1024;
		
		if (eraseFlash(USER_APPLICATION_ADDRESS)) {
			for (uint16_t i = 0; i < MAX_USER_FLASH_SIZE; i++) {
				copyFlashToRam_1K(&flash_source, (uint32_t*)ram);
				if (copyRamToFlash_1K(&flash_destination, (uint32_t*)ram) == 0) {
					gIsFlashSuccess = 0;
					break;
				}

				gIsFlashSuccess = 1;
				GPIO_WriteBit(GPIOC, LED_GPIO_PIN, (BitAction)(1-GPIO_ReadOutputDataBit(GPIOC, LED_GPIO_PIN)));
				msDelay(50);
			}
		}
	}
	
	if (gIsFlashSuccess == 1) {
		if (((*(__IO uint32_t*)USER_APPLICATION_ADDRESS) & 0x2FFE0000 ) == 0x20000000) {
			// Jump to user application
			JumpAddress = *(__IO uint32_t*) (USER_APPLICATION_ADDRESS + 4);
			Jump_To_Application = (pFunction) JumpAddress;

			// Initialize user application's Stack Pointer
			__set_MSP(*(__IO uint32_t*) USER_APPLICATION_ADDRESS);
			msDelay(1000);
			Jump_To_Application();
		}
	}

	while(1)
	{}
}