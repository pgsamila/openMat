/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsConfig.h"

__IO uint32_t SectorsWRPStatus = 0xFFF;

extern LpmsReg gReg;

uint8_t writeRamToFlash(__IO uint32_t* flashAddress, uint32_t* ramBuffer, uint32_t length)
{	
	for (uint16_t i = 0; i < length; i++) {
		if (FLASH_ProgramWord(*flashAddress, *(uint32_t*)(ramBuffer + i)) == FLASH_COMPLETE) {
			if (*(uint32_t*)*flashAddress != *(uint32_t*)(ramBuffer + i)) {
				return 0;
			}

			*flashAddress += 4;
		} else {
			return 0;
		}
	}
	
	return 1;
}

uint8_t eraseCompleteRegisterSet(void)
{
	FLASH_Unlock();	
	
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	
	if (FLASH_EraseSector(FLASH_Sector_6, VoltageRange_3) != FLASH_COMPLETE) {
		return 0;
	}

	return 1;
}

uint8_t writeCompleteRegisterSet(void)
{
  	uint32_t address, buffer;
	buffer = (uint32_t)gReg.data;
	
	FLASH_Unlock();	
	
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	
	if (FLASH_EraseSector(FLASH_Sector_6, VoltageRange_3) != FLASH_COMPLETE) {
		return 0;
	}
	
	address = USER_FLASH_START_ADDRESS;
	if (writeRamToFlash(&address, (uint32_t*)buffer, REG_ARRAY_SIZE) == 0) {
		return 0;
	}
	
	return 1;
}

void loadFlashToRam(__IO uint32_t* flashAddress, uint32_t* ramBuffer, uint32_t length)
{	
	for (uint16_t i = 0; i < length; i++) {
		*(uint32_t*)(ramBuffer + i) = *(uint32_t*)*flashAddress;
		*flashAddress += 4;
	}
}

uint8_t resetToFactory(void)
{
	float2int f2int;
	
	FLASH_Unlock();
	
	SectorsWRPStatus = FLASH_OB_GetWRP() & (OB_WRP_Sector_6);
	if (SectorsWRPStatus == 0x00) {
	  	FLASH_OB_Unlock();
		FLASH_OB_WRPConfig(OB_WRP_Sector_6, DISABLE);
		if (FLASH_OB_Launch() != FLASH_COMPLETE) {
			return 0;
		}
		 FLASH_OB_Lock();
	}
	
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	gReg.data[LPMS_CONFIG] = LPMS_FACTORY_CONFIG;
	gReg.data[LPMS_IMU_ID] = LPMS_FACTORY_IMU_ID;
	
	gReg.data[LPMS_GYR_RANGE] = LPMS_FACTORY_GYR_RANGE;
	gReg.data[LPMS_ACC_RANGE] = LPMS_FACTORY_ACC_RANGE;
	gReg.data[LPMS_MAG_RANGE] = LPMS_FACTORY_MAG_RANGE;
	
	gReg.data[LPMS_GYR_OUTPUT_RATE] = LPMS_FACTORY_GYR_OUTPUT_RATE;
	gReg.data[LPMS_ACC_OUTPUT_RATE] = LPMS_FACTORY_ACC_OUTPUT_RATE;
	gReg.data[LPMS_MAG_OUTPUT_RATE] = LPMS_FACTORY_MAG_OUTPUT_RATE;
	
	f2int.float_val = (float)LPMS_FACTORY_GYR_BIAS_X;
	gReg.data[LPMS_GYR_BIAS_X] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_BIAS_Y;
	gReg.data[LPMS_GYR_BIAS_Y] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_BIAS_Z;
	gReg.data[LPMS_GYR_BIAS_Z] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_GAIN_X;
	gReg.data[LPMS_GYR_GAIN_X] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_GAIN_Y;
	gReg.data[LPMS_GYR_GAIN_Y] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_GAIN_Z;
	gReg.data[LPMS_GYR_GAIN_Z] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_ACC_BIAS_X;
	gReg.data[LPMS_ACC_BIAS_X] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_BIAS_Y;
	gReg.data[LPMS_ACC_BIAS_Y] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_BIAS_Z;
	gReg.data[LPMS_ACC_BIAS_Z] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_GAIN_X;
	gReg.data[LPMS_ACC_GAIN_X] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_GAIN_Y;
	gReg.data[LPMS_ACC_GAIN_Y] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_GAIN_Z;
	gReg.data[LPMS_ACC_GAIN_Z] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_MAG_BIAS_X;
	gReg.data[LPMS_MAG_BIAS_X] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_BIAS_Y;
	gReg.data[LPMS_MAG_BIAS_Y] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_BIAS_Z;
	gReg.data[LPMS_MAG_BIAS_Z] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_GAIN_X;
	gReg.data[LPMS_MAG_GAIN_X] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_GAIN_Y;
	gReg.data[LPMS_MAG_GAIN_Y] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_GAIN_Z;
	gReg.data[LPMS_MAG_GAIN_Z] = f2int.u32_val;

	f2int.float_val = (float)LPMS_FACTORY_MAG_SOFT_00;
	gReg.data[LPMS_MAG_SOFT_00] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_SOFT_01;
	gReg.data[LPMS_MAG_SOFT_01] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_SOFT_02;
	gReg.data[LPMS_MAG_SOFT_02] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_SOFT_10;
	gReg.data[LPMS_MAG_SOFT_10] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_SOFT_11;
	gReg.data[LPMS_MAG_SOFT_11] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_SOFT_12;
	gReg.data[LPMS_MAG_SOFT_12] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_SOFT_20;
	gReg.data[LPMS_MAG_SOFT_20] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_SOFT_21;
	gReg.data[LPMS_MAG_SOFT_21] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_SOFT_22;
	gReg.data[LPMS_MAG_SOFT_22] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_GYR_ALIG_00;
	gReg.data[LPMS_GYR_ALIG_00] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_ALIG_01;
	gReg.data[LPMS_GYR_ALIG_01] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_ALIG_02;
	gReg.data[LPMS_GYR_ALIG_02] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_ALIG_10;
	gReg.data[LPMS_GYR_ALIG_10] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_ALIG_11;
	gReg.data[LPMS_GYR_ALIG_11] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_ALIG_12;
	gReg.data[LPMS_GYR_ALIG_12] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_ALIG_20;
	gReg.data[LPMS_GYR_ALIG_20] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_ALIG_21;
	gReg.data[LPMS_GYR_ALIG_21] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_ALIG_22;
	gReg.data[LPMS_GYR_ALIG_22] = f2int.u32_val;

	f2int.float_val = (float) LPMS_FACTORY_GYR_ALIG_BIAS_X;
	gReg.data[LPMS_GYR_ALIG_BIAS_X] = f2int.u32_val;
	f2int.float_val = (float) LPMS_FACTORY_GYR_ALIG_BIAS_Y;
	gReg.data[LPMS_GYR_ALIG_BIAS_Y] = f2int.u32_val;
	f2int.float_val = (float) LPMS_FACTORY_GYR_ALIG_BIAS_Z;
	gReg.data[LPMS_GYR_ALIG_BIAS_Z] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_ACC_ALIG_00;
	gReg.data[LPMS_ACC_ALIG_00] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_ALIG_01;
	gReg.data[LPMS_ACC_ALIG_01] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_ALIG_02;
	gReg.data[LPMS_ACC_ALIG_02] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_ALIG_10;
	gReg.data[LPMS_ACC_ALIG_10] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_ALIG_11;
	gReg.data[LPMS_ACC_ALIG_11] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_ALIG_12;
	gReg.data[LPMS_ACC_ALIG_12] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_ALIG_20;
	gReg.data[LPMS_ACC_ALIG_20] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_ALIG_21;
	gReg.data[LPMS_ACC_ALIG_21] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_ALIG_22;
	gReg.data[LPMS_ACC_ALIG_22] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_MAG_ALIG_00;
	gReg.data[LPMS_MAG_ALIG_00] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_ALIG_01;
	gReg.data[LPMS_MAG_ALIG_01] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_ALIG_02;
	gReg.data[LPMS_MAG_ALIG_02] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_ALIG_10;
	gReg.data[LPMS_MAG_ALIG_10] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_ALIG_11;
	gReg.data[LPMS_MAG_ALIG_11] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_ALIG_12;
	gReg.data[LPMS_MAG_ALIG_12] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_ALIG_20;
	gReg.data[LPMS_MAG_ALIG_20] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_ALIG_21;
	gReg.data[LPMS_MAG_ALIG_21] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_ALIG_22;
	gReg.data[LPMS_MAG_ALIG_22] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_ACC_REF_X;
	gReg.data[LPMS_ACC_REF_X] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_REF_Y;
	gReg.data[LPMS_ACC_REF_Y] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_REF_Z;
	gReg.data[LPMS_ACC_REF_Z] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_MAG_REF_X;
	gReg.data[LPMS_MAG_REF_X] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_REF_Y;
	gReg.data[LPMS_MAG_REF_Y] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_REF_Z;
	gReg.data[LPMS_MAG_REF_Z] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_GYR_THRES_X;
	gReg.data[LPMS_GYR_THRES_X] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_THRES_Y;
	gReg.data[LPMS_GYR_THRES_Y] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_GYR_THRES_Z;
	gReg.data[LPMS_GYR_THRES_Z] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_THRES_X;
	gReg.data[LPMS_MAG_THRES_X] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_THRES_Y;
	gReg.data[LPMS_MAG_THRES_Y] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_THRES_Z;
	gReg.data[LPMS_MAG_THRES_Z] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_ACC_COMP_GAIN_1;
	gReg.data[LPMS_ACC_COMP_GAIN_1] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_COVAR_1;
	gReg.data[LPMS_ACC_COVAR_1] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_COMP_GAIN_1;
	gReg.data[LPMS_MAG_COMP_GAIN_1] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_COVAR_1;
	gReg.data[LPMS_MAG_COVAR_1] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_ACC_COMP_GAIN_2;
	gReg.data[LPMS_ACC_COMP_GAIN_2] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_COVAR_2;
	gReg.data[LPMS_ACC_COVAR_2] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_COMP_GAIN_2;
	gReg.data[LPMS_MAG_COMP_GAIN_2] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_COVAR_2;
	gReg.data[LPMS_MAG_COVAR_2] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_ACC_COMP_GAIN_3;
	gReg.data[LPMS_ACC_COMP_GAIN_3] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_COVAR_3;
	gReg.data[LPMS_ACC_COVAR_3] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_COMP_GAIN_3;
	gReg.data[LPMS_MAG_COMP_GAIN_3] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_COVAR_3;
	gReg.data[LPMS_MAG_COVAR_3] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_ACC_COMP_GAIN_USER;
	gReg.data[LPMS_ACC_COMP_GAIN_USER] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_ACC_COVAR_USER;
	gReg.data[LPMS_ACC_COVAR_USER] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_COMP_GAIN_USER;
	gReg.data[LPMS_MAG_COMP_GAIN_USER] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_MAG_COVAR_USER;
	gReg.data[LPMS_MAG_COVAR_USER] = f2int.u32_val;
	
	f2int.float_val = (float)LPMS_FACTORY_PROCESS_COVAR;
	gReg.data[LPMS_PROCESS_COVAR] = f2int.u32_val;
			 
	f2int.float_val = (float)LPMS_FACTORY_OFFSET_QUAT_0; 
	gReg.data[LPMS_OFFSET_QUAT_0] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_OFFSET_QUAT_1; 
	gReg.data[LPMS_OFFSET_QUAT_1] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_OFFSET_QUAT_2; 
	gReg.data[LPMS_OFFSET_QUAT_2] = f2int.u32_val;
	f2int.float_val = (float)LPMS_FACTORY_OFFSET_QUAT_3; 
	gReg.data[LPMS_OFFSET_QUAT_3] = f2int.u32_val;
                 
	gReg.data[LPMS_FILTER_MODE] = LPMS_FILTER_GYR_ACC_MAG;
	gReg.data[LPMS_FILTER_PRESET] = LPMS_FILTER_PRM_SET_4;

	gReg.data[LPMS_MAG_FIELD_EST] = conFtoI(LPMS_FACTORY_FIELD_EST);
	gReg.data[LPMS_MAG_FIELD_INC] = LPMS_FACTORY_FIELD_INC;

	gReg.data[LPMS_RAW_DATA_LP] = LPMS_FACTORY_RAW_DATA_LP;

	gReg.data[LPMS_CAN_MAPPING] = LPMS_FACTORY_CAN_MAPPING_0;
	gReg.data[LPMS_CAN_MAPPING+1] = LPMS_FACTORY_CAN_MAPPING_1;
	gReg.data[LPMS_CAN_MAPPING+2] = LPMS_FACTORY_CAN_MAPPING_2;
	gReg.data[LPMS_CAN_MAPPING+3] = LPMS_FACTORY_CAN_MAPPING_3;
	gReg.data[LPMS_CAN_MAPPING+4] = LPMS_FACTORY_CAN_MAPPING_4;
	gReg.data[LPMS_CAN_MAPPING+5] = LPMS_FACTORY_CAN_MAPPING_5;
	gReg.data[LPMS_CAN_MAPPING+6] = LPMS_FACTORY_CAN_MAPPING_6;
	gReg.data[LPMS_CAN_MAPPING+7] = LPMS_FACTORY_CAN_MAPPING_7;
        gReg.data[LPMS_CAN_MAPPING+8] = LPMS_FACTORY_CAN_MAPPING_8;
	gReg.data[LPMS_CAN_MAPPING+9] = LPMS_FACTORY_CAN_MAPPING_9;
	gReg.data[LPMS_CAN_MAPPING+10] = LPMS_FACTORY_CAN_MAPPING_10;
	gReg.data[LPMS_CAN_MAPPING+11] = LPMS_FACTORY_CAN_MAPPING_11;
	gReg.data[LPMS_CAN_MAPPING+12] = LPMS_FACTORY_CAN_MAPPING_12;
	gReg.data[LPMS_CAN_MAPPING+13] = LPMS_FACTORY_CAN_MAPPING_13;
	gReg.data[LPMS_CAN_MAPPING+14] = LPMS_FACTORY_CAN_MAPPING_14;
	gReg.data[LPMS_CAN_MAPPING+15] = LPMS_FACTORY_CAN_MAPPING_15;

	gReg.data[LPMS_CAN_HEARTBEAT] = LPMS_FACTORY_CAN_HEARTBEAT;     

	gReg.data[LPMS_LIN_ACC_COMP_MODE] = LPMS_FACTORY_LIN_ACC_COMP_MODE;
	gReg.data[LPMS_CENTRI_COMP_MODE] = LPMS_FACTORY_CENTRI_COMP_MODE;

	gReg.data[LPMS_FILTER_MODE] = LPMS_FACTORY_FILTER_MODE;

	gReg.data[LPMS_CAN_CONFIGURATION] = LPMS_FACTORY_CAN_CONFIGURATION;

	writeCompleteRegisterSet();
	
	return 1;
}

void copyFlashToRam_256bytes(__IO uint32_t* flashAddress, uint32_t* ramBuffer)
{
	uint16_t data_length;
	
	data_length = 256/4;
	
	for (uint16_t i = 0; i < data_length; i++ ) {
		*(uint32_t*)(ramBuffer + i) = *(uint32_t*)*flashAddress;
		*flashAddress += 4;
	}
}

uint8_t copyRamToFlash_256bytes(__IO uint32_t* flashAddress, uint32_t* ramBuffer)
{
	uint16_t data_length;
	uint32_t temp1=0;
	uint32_t temp2=0;
	
	FLASH_Unlock();
	
	SectorsWRPStatus = FLASH_OB_GetWRP() & (OB_WRP_Sector_5 | OB_WRP_Sector_7);
	if (SectorsWRPStatus == 0x00) {
	  	FLASH_OB_Unlock();
		FLASH_OB_WRPConfig((OB_WRP_Sector_5 | OB_WRP_Sector_7), DISABLE);
		if (FLASH_OB_Launch() != FLASH_COMPLETE) {
			return 0;
		}
		FLASH_OB_Lock();
	}

	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	
	data_length = 256/4;
	
	for (uint16_t i = 0; i < data_length; i++) {
		if (FLASH_ProgramWord(*flashAddress, *(uint32_t*)(ramBuffer + i)) == FLASH_COMPLETE) {
		  	temp1 = *(__IO uint32_t*)(*flashAddress);
			temp2 = *(uint32_t*)(ramBuffer + i);
		  	
			if (temp1 != temp2) {
				return 0;
			}

			*flashAddress += 4;
		} else {
			return 0;
		}
	}
	
	return 1;
}

uint8_t copyRamToFlash_128bytes(__IO uint32_t* flashAddress, uint32_t* ramBuffer)
{
	uint16_t data_length;
	uint32_t temp1=0;
	uint32_t temp2=0;
	
	FLASH_Unlock();
	
	SectorsWRPStatus = FLASH_OB_GetWRP() & (OB_WRP_Sector_5 | OB_WRP_Sector_7);
	if (SectorsWRPStatus == 0x00) {
	  	FLASH_OB_Unlock();
		FLASH_OB_WRPConfig((OB_WRP_Sector_5 | OB_WRP_Sector_7), DISABLE);
		if (FLASH_OB_Launch() != FLASH_COMPLETE) {
			return 0;
		}
		FLASH_OB_Lock();
	}

	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	
	data_length = 128/4;
	
	for (uint16_t i = 0; i < data_length; i++) {
		if (FLASH_ProgramWord(*flashAddress, *(uint32_t*)(ramBuffer + i)) == FLASH_COMPLETE) {
		  	temp1 = *(__IO uint32_t*)(*flashAddress);
			temp2 = *(uint32_t*)(ramBuffer + i);
		  	
			if (temp1 != temp2) {
				return 0;
			}

			*flashAddress += 4;
		} else {
			return 0;
		}
	}
	
	return 1;
}