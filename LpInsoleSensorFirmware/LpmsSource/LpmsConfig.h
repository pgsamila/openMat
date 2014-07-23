/***********************************************************************
** Sensor register definitions and other configuration related things
**
** Copyright (C) 2013 LP-RESEARCH Inc.
** All rights reserved.
** Contact: info@lp-research.com
***********************************************************************/

#ifndef LPMS_CONFIG_H
#define LPMS_CONFIG_H

#include "stm32f2xx.h"
#include "LpmsFactorySetting.h"
#include "LpMatrix.h"
#include "LpmsAssignments.h"

// Release information
#define	LPMS_FIRMWARE_REVISION 		('v' << 24) | ('0' << 16) | ('0' << 8) | '1')
#define LPMS_DEVICE_ID_FIRST_PART 	(('L' << 24) | ('P' << 16) | ('M' << 8) | 'S')
#define LPMS_DEVICE_ID_SECOND_PART 	(('-' << 24) | ('B' << 16) | ('x' << 8) | 'x')
#define LPMS_DEVICE_RELEASE_DATE 	(('1' << 24) | ('2' << 16) | ('0' << 8) | '3') 

// Size of register array
#define	REG_ARRAY_SIZE 		(uint32_t)137

// Number of commands
#define	COMMAND_ARRAY_SIZE 	(uint32_t)96

// Register start address
#define	REG_START_ADDRESS 	0

// Commands start address
#define	COMMAND_START_ADDRESS 	0

// Registers
#define	LPMS_CONFIG 		REG_START_ADDRESS
#define LPMS_IMU_ID 		(REG_START_ADDRESS + 1)
#define LPMS_GYR_RANGE		(REG_START_ADDRESS + 2)
#define LPMS_ACC_RANGE 		(REG_START_ADDRESS + 3)
#define LPMS_MAG_RANGE 		(REG_START_ADDRESS + 4)
#define LPMS_GYR_OUTPUT_RATE 	(REG_START_ADDRESS + 5)
#define LPMS_ACC_OUTPUT_RATE 	(REG_START_ADDRESS + 6)
#define LPMS_MAG_OUTPUT_RATE 	(REG_START_ADDRESS + 7)
#define	LPMS_GYR_BIAS_X 	(REG_START_ADDRESS + 8)
#define	LPMS_GYR_BIAS_Y 	(REG_START_ADDRESS + 9)
#define	LPMS_GYR_BIAS_Z 	(REG_START_ADDRESS + 10)
#define	LPMS_GYR_GAIN_X 	(REG_START_ADDRESS + 11)
#define	LPMS_GYR_GAIN_Y 	(REG_START_ADDRESS + 12)
#define	LPMS_GYR_GAIN_Z 	(REG_START_ADDRESS + 13)
#define	LPMS_ACC_BIAS_X 	(REG_START_ADDRESS + 14)
#define	LPMS_ACC_BIAS_Y 	(REG_START_ADDRESS + 15)
#define	LPMS_ACC_BIAS_Z 	(REG_START_ADDRESS + 16)
#define	LPMS_ACC_GAIN_X 	(REG_START_ADDRESS + 17)
#define	LPMS_ACC_GAIN_Y 	(REG_START_ADDRESS + 18)
#define	LPMS_ACC_GAIN_Z 	(REG_START_ADDRESS + 19)
#define	LPMS_MAG_BIAS_X 	(REG_START_ADDRESS + 20)
#define	LPMS_MAG_BIAS_Y 	(REG_START_ADDRESS + 21)
#define	LPMS_MAG_BIAS_Z 	(REG_START_ADDRESS + 22)
#define	LPMS_MAG_GAIN_X 	(REG_START_ADDRESS + 23)
#define	LPMS_MAG_GAIN_Y 	(REG_START_ADDRESS + 24)
#define	LPMS_MAG_GAIN_Z 	(REG_START_ADDRESS + 25)
#define	LPMS_GYR_ALIG_00 	(REG_START_ADDRESS + 26)
#define	LPMS_GYR_ALIG_01 	(REG_START_ADDRESS + 27)
#define	LPMS_GYR_ALIG_02 	(REG_START_ADDRESS + 28)
#define	LPMS_GYR_ALIG_10 	(REG_START_ADDRESS + 29)
#define	LPMS_GYR_ALIG_11 	(REG_START_ADDRESS + 30)
#define	LPMS_GYR_ALIG_12 	(REG_START_ADDRESS + 31)
#define	LPMS_GYR_ALIG_20 	(REG_START_ADDRESS + 32)
#define	LPMS_GYR_ALIG_21 	(REG_START_ADDRESS + 33)
#define	LPMS_GYR_ALIG_22 	(REG_START_ADDRESS + 34)
#define	LPMS_ACC_ALIG_00 	(REG_START_ADDRESS + 35)
#define	LPMS_ACC_ALIG_01 	(REG_START_ADDRESS + 36)
#define	LPMS_ACC_ALIG_02 	(REG_START_ADDRESS + 37)
#define	LPMS_ACC_ALIG_10 	(REG_START_ADDRESS + 38)
#define	LPMS_ACC_ALIG_11 	(REG_START_ADDRESS + 39)
#define	LPMS_ACC_ALIG_12 	(REG_START_ADDRESS + 40)
#define	LPMS_ACC_ALIG_20 	(REG_START_ADDRESS + 41)
#define	LPMS_ACC_ALIG_21 	(REG_START_ADDRESS + 42)
#define	LPMS_ACC_ALIG_22 	(REG_START_ADDRESS + 43)
#define	LPMS_MAG_ALIG_00 	(REG_START_ADDRESS + 44)
#define	LPMS_MAG_ALIG_01 	(REG_START_ADDRESS + 45)
#define	LPMS_MAG_ALIG_02 	(REG_START_ADDRESS + 46)
#define	LPMS_MAG_ALIG_10 	(REG_START_ADDRESS + 47)
#define	LPMS_MAG_ALIG_11 	(REG_START_ADDRESS + 48)
#define	LPMS_MAG_ALIG_12 	(REG_START_ADDRESS + 49)
#define	LPMS_MAG_ALIG_20 	(REG_START_ADDRESS + 50)
#define	LPMS_MAG_ALIG_21 	(REG_START_ADDRESS + 51)
#define	LPMS_MAG_ALIG_22 	(REG_START_ADDRESS + 52)
#define	LPMS_ACC_REF_X 		(REG_START_ADDRESS + 53)
#define	LPMS_ACC_REF_Y 		(REG_START_ADDRESS + 54)
#define	LPMS_ACC_REF_Z 		(REG_START_ADDRESS + 55)
#define	LPMS_MAG_REF_X 		(REG_START_ADDRESS + 56)
#define	LPMS_MAG_REF_Y 		(REG_START_ADDRESS + 57)
#define	LPMS_MAG_REF_Z 		(REG_START_ADDRESS + 58)
#define LPMS_GYR_THRES_X 	(REG_START_ADDRESS + 59)
#define LPMS_GYR_THRES_Y 	(REG_START_ADDRESS + 60)
#define LPMS_GYR_THRES_Z 	(REG_START_ADDRESS + 61)
#define LPMS_MAG_THRES_X 	(REG_START_ADDRESS + 62)
#define LPMS_MAG_THRES_Y 	(REG_START_ADDRESS + 63)
#define LPMS_MAG_THRES_Z 	(REG_START_ADDRESS + 64)
#define LPMS_ACC_COMP_GAIN_1 	(REG_START_ADDRESS + 65)
#define LPMS_ACC_COVAR_1 	(REG_START_ADDRESS + 66)
#define LPMS_MAG_COMP_GAIN_1 	(REG_START_ADDRESS + 67)
#define LPMS_MAG_COVAR_1 	(REG_START_ADDRESS + 68)
#define LPMS_ACC_COMP_GAIN_2 	(REG_START_ADDRESS + 69)
#define LPMS_ACC_COVAR_2 	(REG_START_ADDRESS + 70)
#define LPMS_MAG_COMP_GAIN_2 	(REG_START_ADDRESS + 71)
#define LPMS_MAG_COVAR_2 	(REG_START_ADDRESS + 72)
#define LPMS_ACC_COMP_GAIN_3 	(REG_START_ADDRESS + 73)
#define LPMS_ACC_COVAR_3 	(REG_START_ADDRESS + 74)
#define LPMS_MAG_COMP_GAIN_3 	(REG_START_ADDRESS + 75)
#define LPMS_MAG_COVAR_3 	(REG_START_ADDRESS + 76)
#define LPMS_ACC_COMP_GAIN_USER (REG_START_ADDRESS + 77)
#define LPMS_ACC_COVAR_USER 	(REG_START_ADDRESS + 78)
#define LPMS_MAG_COMP_GAIN_USER (REG_START_ADDRESS + 79)
#define LPMS_MAG_COVAR_USER 	(REG_START_ADDRESS + 80)
#define LPMS_PROCESS_COVAR 	(REG_START_ADDRESS + 81)
#define LPMS_FILTER_MODE 	(REG_START_ADDRESS + 82)
#define LPMS_FILTER_PRESET 	(REG_START_ADDRESS + 83)
#define LPMS_OFFSET_QUAT_0 	(REG_START_ADDRESS + 84)
#define LPMS_OFFSET_QUAT_1 	(REG_START_ADDRESS + 85)
#define LPMS_OFFSET_QUAT_2 	(REG_START_ADDRESS + 86)
#define LPMS_OFFSET_QUAT_3 	(REG_START_ADDRESS + 87)
#define LPMS_MAG_SOFT_00 	(REG_START_ADDRESS + 88)
#define LPMS_MAG_SOFT_01 	(REG_START_ADDRESS + 89)
#define LPMS_MAG_SOFT_02 	(REG_START_ADDRESS + 90)
#define LPMS_MAG_SOFT_10 	(REG_START_ADDRESS + 91)
#define LPMS_MAG_SOFT_11 	(REG_START_ADDRESS + 92)
#define LPMS_MAG_SOFT_12 	(REG_START_ADDRESS + 93)
#define LPMS_MAG_SOFT_20 	(REG_START_ADDRESS + 94)
#define LPMS_MAG_SOFT_21 	(REG_START_ADDRESS + 95)
#define LPMS_MAG_SOFT_22 	(REG_START_ADDRESS + 96)
#define LPMS_MAG_DECLINATION 	(REG_START_ADDRESS + 97)
#define LPMS_MAG_FIELD_EST 	(REG_START_ADDRESS + 98)
#define LPMS_MAG_FIELD_INC 	(REG_START_ADDRESS + 99)
#define	LPMS_GYR_ALIG_BIAS_X 	(REG_START_ADDRESS + 100)
#define	LPMS_GYR_ALIG_BIAS_Y 	(REG_START_ADDRESS + 101)
#define	LPMS_GYR_ALIG_BIAS_Z	(REG_START_ADDRESS + 102)
#define LPMS_RAW_DATA_LP 	(REG_START_ADDRESS + 113)
#define LPMS_CAN_MAPPING 	(REG_START_ADDRESS + 114)
#define LPMS_CAN_HEARTBEAT 	(REG_START_ADDRESS + 130)
#define LPMS_LIN_ACC_COMP_MODE 	(REG_START_ADDRESS + 131)
#define LPMS_CENTRI_COMP_MODE 	(REG_START_ADDRESS + 132)
#define LPMS_CAN_CONFIGURATION 	(REG_START_ADDRESS + 133)
#define	LPMS_MAG_ALIG_BIAS_X 	(REG_START_ADDRESS + 134)
#define	LPMS_MAG_ALIG_BIAS_Y 	(REG_START_ADDRESS + 135)
#define	LPMS_MAG_ALIG_BIAS_Z 	(REG_START_ADDRESS + 136)

// Commands --> 

// Acknowledged and Not-acknowledged identifier
#define REPLY_ACK 			(COMMAND_START_ADDRESS + 0)
#define REPLY_NACK 			(COMMAND_START_ADDRESS + 1)

// Firmware update and in-application-programmer upload
#define UPDATE_FIRMWARE 		(COMMAND_START_ADDRESS + 2)	
#define UPDATE_IAP 			(COMMAND_START_ADDRESS + 3)	

// Configuration and status
#define GET_CONFIG 			(COMMAND_START_ADDRESS + 4)	
#define GET_STATUS 			(COMMAND_START_ADDRESS + 5)	

// Mode switching
#define GOTO_COMMAND_MODE 		(COMMAND_START_ADDRESS + 6)	
#define GOTO_STREAM_MODE 		(COMMAND_START_ADDRESS + 7)	
#define GOTO_SLEEP_MODE 		(COMMAND_START_ADDRESS + 8)	

// Data transmission
#define GET_SENSOR_DATA 		(COMMAND_START_ADDRESS + 9)
#define SET_TRANSMIT_DATA 		(COMMAND_START_ADDRESS + 10)
#define SET_STREAM_FREQ 		(COMMAND_START_ADDRESS + 11)	
#define GET_ROLL 			(COMMAND_START_ADDRESS + 12)
#define GET_PITCH 			(COMMAND_START_ADDRESS + 13)
#define GET_YAW 			(COMMAND_START_ADDRESS + 14)

// Commands
// Register value save and reset
#define WRITE_REGISTERS 		(COMMAND_START_ADDRESS + 15)
#define RESTORE_FACTORY_VALUE 		(COMMAND_START_ADDRESS + 16)	

// Reference setting and offset reset 
#define RESET_REFERENCE 		(COMMAND_START_ADDRESS + 17)	
#define SET_ORIENTATION_OFFSET		(COMMAND_START_ADDRESS + 18)	

// Self-test
#define SELF_TEST 			(COMMAND_START_ADDRESS + 19)	

// IMU ID setting
#define SET_IMU_ID 			(COMMAND_START_ADDRESS + 20)	
#define GET_IMU_ID 			(COMMAND_START_ADDRESS + 21)	

// Gyroscope settings
#define START_GYR_CALIBRA 		(COMMAND_START_ADDRESS + 22)	
#define ENABLE_GYR_AUTOCAL 		(COMMAND_START_ADDRESS + 23)	
#define ENABLE_GYR_THRES 		(COMMAND_START_ADDRESS + 24)	
#define SET_GYR_RANGE 			(COMMAND_START_ADDRESS + 25)	
#define GET_GYR_RANGE 			(COMMAND_START_ADDRESS + 26)	

// Accelerometer settings
#define SET_ACC_BIAS 			(COMMAND_START_ADDRESS + 27)	
#define GET_ACC_BIAS 			(COMMAND_START_ADDRESS + 28)	
#define SET_ACC_ALIGN_MATRIX		(COMMAND_START_ADDRESS + 29)	
#define GET_ACC_ALIGN_MATRIX 		(COMMAND_START_ADDRESS + 30)	
#define SET_ACC_RANGE 			(COMMAND_START_ADDRESS + 31)	
#define GET_ACC_RANGE 			(COMMAND_START_ADDRESS + 32)	

// Magnetometer settings
#define SET_MAG_RANGE 			(COMMAND_START_ADDRESS + 33)	
#define GET_MAG_RANGE 			(COMMAND_START_ADDRESS + 34)	
#define SET_HARD_IRON_OFFSET 		(COMMAND_START_ADDRESS + 35)	
#define GET_HARD_IRON_OFFSET 		(COMMAND_START_ADDRESS + 36)	
#define SET_SOFT_IRON_MATRIX 		(COMMAND_START_ADDRESS + 37)	
#define GET_SOFT_IRON_MATRIX 		(COMMAND_START_ADDRESS + 38)
#define SET_FIELD_ESTIMATE 		(COMMAND_START_ADDRESS + 39)	
#define GET_FIELD_ESTIMATE 		(COMMAND_START_ADDRESS + 40)

// Filter settings
#define SET_FILTER_MODE 		(COMMAND_START_ADDRESS + 41)	
#define GET_FILTER_MODE 		(COMMAND_START_ADDRESS + 42)	
#define SET_FILTER_PRESET 		(COMMAND_START_ADDRESS + 43)	
#define GET_FILTER_PRESET 		(COMMAND_START_ADDRESS + 44)	

// CAN settings
#define SET_CAN_STREAM_FORMAT 		(COMMAND_START_ADDRESS + 45)	
#define SET_CAN_BAUDRATE 		(COMMAND_START_ADDRESS + 46)

// Additional parameters
#define GET_FIRMWARE_VERSION		(COMMAND_START_ADDRESS + 47)
#define SET_GYR_ALIGN_BIAS 		(COMMAND_START_ADDRESS + 48)	
#define GET_GYR_ALIGN_BIAS 		(COMMAND_START_ADDRESS + 49)	
#define SET_GYR_ALIGN_MATRIX		(COMMAND_START_ADDRESS + 50)	
#define GET_GYR_ALIGN_MATRIX 		(COMMAND_START_ADDRESS + 51)	
#define SET_GYR_TEMP_CAL_PRM_A		(COMMAND_START_ADDRESS + 52)	
#define GET_GYR_TEMP_CAL_PRM_A		(COMMAND_START_ADDRESS + 53)
#define SET_GYR_TEMP_CAL_PRM_B		(COMMAND_START_ADDRESS + 54)	
#define GET_GYR_TEMP_CAL_PRM_B		(COMMAND_START_ADDRESS + 55)	
#define SET_GYR_TEMP_CAL_BASE_V		(COMMAND_START_ADDRESS + 56)	
#define GET_GYR_TEMP_CAL_BASE_V		(COMMAND_START_ADDRESS + 57)	
#define SET_GYR_TEMP_CAL_BASE_T		(COMMAND_START_ADDRESS + 58)	
#define GET_GYR_TEMP_CAL_BASE_T		(COMMAND_START_ADDRESS + 59)
#define SET_RAW_DATA_LP 		(COMMAND_START_ADDRESS + 60)
#define GET_RAW_DATA_LP 		(COMMAND_START_ADDRESS + 61)
#define SET_CAN_MAPPING 		(COMMAND_START_ADDRESS + 62)
#define GET_CAN_MAPPING 		(COMMAND_START_ADDRESS + 63)
#define SET_CAN_HEARTBEAT		(COMMAND_START_ADDRESS + 64)
#define GET_CAN_HEARTBEAT		(COMMAND_START_ADDRESS + 65)
#define SET_TIMESTAMP			(COMMAND_START_ADDRESS + 66)
#define SET_LIN_ACC_COMP_MODE		(COMMAND_START_ADDRESS + 67)
#define GET_LIN_ACC_COMP_MODE		(COMMAND_START_ADDRESS + 68)
#define SET_CENTRI_COMP_MODE		(COMMAND_START_ADDRESS + 69)
#define GET_CENTRI_COMP_MODE		(COMMAND_START_ADDRESS + 70)
#define GET_CAN_CONFIGURATION		(COMMAND_START_ADDRESS + 71)
#define SET_CAN_CHANNEL_MODE 		(COMMAND_START_ADDRESS + 72)
#define SET_CAN_POINT_MODE 		(COMMAND_START_ADDRESS + 73)
#define SET_CAN_START_ID 		(COMMAND_START_ADDRESS + 74)
#define SET_LPBUS_DATA_MODE		(COMMAND_START_ADDRESS + 75)
#define SET_MAG_ALIGNMENT_MATRIX	(COMMAND_START_ADDRESS + 76)
#define SET_MAG_ALIGNMENT_BIAS		(COMMAND_START_ADDRESS + 77)
#define SET_MAG_REFRENCE		(COMMAND_START_ADDRESS + 78)
#define GET_MAG_ALIGNMENT_MATRIX	(COMMAND_START_ADDRESS + 79)
#define GET_MAG_ALIGNMENT_BIAS		(COMMAND_START_ADDRESS + 80)
#define GET_MAG_REFERENCE		(COMMAND_START_ADDRESS + 81)
#define RESET_ORIENTATION_OFFSET	(COMMAND_START_ADDRESS + 82)	

// <--

// Adresses for in-application programmer
#define	IAP_FLASH_START_ADDRESS 		(uint32_t)0x08060000
#define IAP_MAX_PACKET_SIZE 			64
#define USER_APPLICATION_BACKUP_ADDRESS 	(uint32_t)0x08020000
#define USER_APPLICATION_MAX_PACKET_SIZE 	512

// User flash addresses
#define	USER_FLASH_START_ADDRESS 	(uint32_t)0x08040000
#define	FACTORY_FLASH_START_ADDRESS 	(uint32_t)0x08050000
#define CHECK_USER_FLASH() 		((uint32_t)( *(__IO u32*)(USER_FLASH_START_ADDRESS) ) != 0xFFFFFFFF)
#define CHECK_FACTORY_FLASH() 		((uint32_t)( *(__IO u32*)(FACTORY_FLASH_START_ADDRESS) ) == 0xFFFFFFFF)

// Configuration register contents
#define NOT_USED_0				(uint32_t)(0x00000001 << 31)
#define LPMS_GYR_AUTOCAL_ENABLED 		(uint32_t)(0x00000001 << 30)
#define NOT_USED_1				(uint32_t)(0x00000001 << 29)
#define NOT_USED_2			 	(uint32_t)(0x00000001 << 28)
#define LPMS_GAIT_TRACKING_ENABLED 		(uint32_t)(0x00000001 << 27)
#define LPMS_HEAVEMOTION_ENABLED 		(uint32_t)(0x00000001 << 26)
#define	LPMS_ACC_COMP_ENABLED 			(uint32_t)(0x00000001 << 25)
#define	LPMS_MAG_COMP_ENABLED 			(uint32_t)(0x00000001 << 24)
#define	LPMS_GYR_THRES_ENABLED 			(uint32_t)(0x00000001 << 23)
#define	LPMS_LPBUS_DATA_MODE_16BIT_ENABLED	(uint32_t)(0x00000001 << 22)
#define	LPMS_LINACC_OUTPUT_ENABLED 		(uint32_t)(0x00000001 << 21)
#define LPMS_DYNAMIC_COVAR_ENABLED 		(uint32_t)(0x00000001 << 20)
#define LPMS_ALTITUDE_OUTPUT_ENABLED 		(uint32_t)(0x00000001 << 19)
#define	LPMS_QUAT_OUTPUT_ENABLED 		(uint32_t)(0x00000001 << 18)
#define	LPMS_EULER_OUTPUT_ENABLED 		(uint32_t)(0x00000001 << 17)
#define	LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED 	(uint32_t)(0x00000001 << 16)
#define	LPMS_GYR_CALIBRA_ENABLED 		(uint32_t)(0x00000001 << 15)
#define	LPMS_HEAVEMOTION_OUTPUT_ENABLED 	(uint32_t)(0x00000001 << 14)
#define	LPMS_TEMPERATURE_OUTPUT_ENABLED		(uint32_t)(0x00000001 << 13)
#define	LPMS_GYR_RAW_OUTPUT_ENABLED		(uint32_t)(0x00000001 << 12)
#define	LPMS_ACC_RAW_OUTPUT_ENABLED		(uint32_t)(0x00000001 << 11)
#define	LPMS_MAG_RAW_OUTPUT_ENABLED		(uint32_t)(0x00000001 << 10)
#define	LPMS_PRESSURE_OUTPUT_ENABLED		(uint32_t)(0x00000001 << 9)

// Status register contents
#define LPMS_COMMAND_MODE 		(0x0001 << 0)
#define LPMS_STREAM_MODE 		(0x0001 << 1)
#define LPMS_SLEEP_MODE			(0x0001 << 2)
#define LPMS_GYR_CALIBRATION_RUNNING 	(0x0001 << 3)
#define LPMS_MAG_CALIBRATION_RUNNING 	(0x0001 << 4)
#define	LPMS_GYR_INIT_FAILED 		(0x0001 << 5)
#define	LPMS_ACC_INIT_FAILED 		(0x0001 << 6)
#define	LPMS_MAG_INIT_FAILED 		(0x0001 << 7)
#define LPMS_PRESSURE_INIT_FAILED 	(0x0001 << 8)
#define	LPMS_GYR_UNRESPONSIVE 		(0x0001 << 9)
#define	LPMS_ACC_UNRESPONSIVE 		(0x0001 << 10)
#define	LPMS_MAG_UNRESPONSIVE 		(0x0001 << 11)
#define	LPMS_FLASH_WRITE_FAILED 	(0x0001 << 12)
#define LPMS_SET_BAUDRATE_FAILED 	(0x0001 << 13)
#define LPMS_SET_BROADCAST_FREQ_FAILED 	(0x0001 << 14)
#define LPMS_REF_CALIBRATION_RUNNING 	(0x0001 << 15)
#define LPMS_SELF_TEST_RUNNING 		(0x0001 << 16)

// Stream frequency enable bits
#define LPMS_STREAM_FREQ_5HZ_ENABLED 	0x00000000
#define LPMS_STREAM_FREQ_10HZ_ENABLED 	0x00000001
#define LPMS_STREAM_FREQ_30HZ_ENABLED 	0x00000002
#define LPMS_STREAM_FREQ_50HZ_ENABLED 	0x00000003
#define LPMS_STREAM_FREQ_100HZ_ENABLED	0x00000004
#define LPMS_STREAM_FREQ_200HZ_ENABLED 	0x00000005
#define LPMS_STREAM_FREQ_500HZ_ENABLED 	0x00000006
#define LPMS_STREAM_FREQ_1000HZ_ENABLED 0x00000007
#define LPMS_STREAM_FREQ_MASK 		0x00000007

// Stream frequency data values
#define LPMS_STREAM_FREQ_5HZ 		(uint32_t)5
#define LPMS_STREAM_FREQ_10HZ 		(uint32_t)10
#define LPMS_STREAM_FREQ_30HZ 		(uint32_t)30
#define LPMS_STREAM_FREQ_50HZ 		(uint32_t)50
#define LPMS_STREAM_FREQ_100HZ 		(uint32_t)100
#define LPMS_STREAM_FREQ_200HZ 		(uint32_t)200
#define LPMS_STREAM_FREQ_300HZ 		(uint32_t)300
#define LPMS_STREAM_FREQ_500HZ 		(uint32_t)500
#define LPMS_STREAM_FREQ_1000HZ 	(uint32_t)1000

// Stream cycle period values
#define LPMS_STREAM_T_5HZ 	(uint32_t)2000
#define LPMS_STREAM_T_10HZ 	(uint32_t)1000
#define LPMS_STREAM_T_20HZ 	(uint32_t)500
#define LPMS_STREAM_T_30HZ 	(uint32_t)334
#define LPMS_STREAM_T_50HZ 	(uint32_t)200
#define LPMS_STREAM_T_100HZ 	(uint32_t)100
#define LPMS_STREAM_T_200HZ 	(uint32_t)50
#define LPMS_STREAM_T_300HZ	(uint32_t)34
#define LPMS_STREAM_T_500HZ 	(uint32_t)20
#define LPMS_STREAM_T_1000HZ 	(uint32_t)10

// Bluetooth baudrate settings
#define LPMS_BT_BAUDRATE_9600_ENABLED 	0x00000000
#define LPMS_BT_BAUDRATE_19200_ENABLED 	0x00000008
#define LPMS_BT_BAUDRATE_38400_ENABLED 	0x00000010
#define LPMS_BT_BAUDRATE_57600_ENABLED 	0x00000018
#define LPMS_BT_BAUDRATE_115200_ENABLED 0x00000020
#define LPMS_BT_BAUDRATE_230400_ENABLED 0x00000028
#define LPMS_BT_BAUDRATE_460800_ENABLED 0x00000030
#define LPMS_BT_BAUDRATE_921600_ENABLED	0x00000038

// CAN bus baudrate settings
#define LPMS_CAN_BAUDRATE_10K_ENABLED 	0x00000000
#define LPMS_CAN_BAUDRATE_20K_ENABLED 	0x00000008
#define LPMS_CAN_BAUDRATE_50K_ENABLED 	0x00000010
#define LPMS_CAN_BAUDRATE_125K_ENABLED 	0x00000018
#define LPMS_CAN_BAUDRATE_250K_ENABLED 	0x00000020
#define LPMS_CAN_BAUDRATE_500K_ENABLED 	0x00000028
#define LPMS_CAN_BAUDRATE_800K_ENABLED 	0x00000030
#define LPMS_CAN_BAUDRATE_1M_ENABLED 	0x00000038

// CAN communication baudrate mask
#define LPMS_CAN_BAUDRATE_MASK 0x00000038

// Serial communication (e.g. Bluetooth) baudrate mask
#define LPMS_BAUDRATE_MASK 0x00000038

// Bluetooth baudrate values
#define LPMS_BT_BAUDRATE_9600 (uint32_t)9600
#define LPMS_BT_BAUDRATE_19200 (uint32_t)19200
#define LPMS_BT_BAUDRATE_38400 (uint32_t)38400
#define LPMS_BT_BAUDRATE_57600 (uint32_t)57600
#define LPMS_BT_BAUDRATE_115200 (uint32_t)115200
#define LPMS_BT_BAUDRATE_230400 (uint32_t)230400
#define LPMS_BT_BAUDRATE_460800 (uint32_t)460800
#define LPMS_BT_BAUDRATE_921600 (uint32_t)921600

// CAN bus baudrate values
#define LPMS_CAN_BAUDRATE_10K 	(uint32_t)10
#define LPMS_CAN_BAUDRATE_20K 	(uint32_t)20
#define LPMS_CAN_BAUDRATE_50K 	(uint32_t)50
#define LPMS_CAN_BAUDRATE_125K 	(uint32_t)125
#define LPMS_CAN_BAUDRATE_250K 	(uint32_t)250
#define LPMS_CAN_BAUDRATE_500K 	(uint32_t)500
#define LPMS_CAN_BAUDRATE_800K 	(uint32_t)800
#define LPMS_CAN_BAUDRATE_1M 	(uint32_t)1000

// LpFilter parameter sets
#define LPMS_FILTER_PRM_SET_1 	0x00000000
#define LPMS_FILTER_PRM_SET_2 	0x00000001
#define LPMS_FILTER_PRM_SET_3 	0x00000002
#define LPMS_FILTER_PRM_SET_4 	0x00000003

// Gyroscope threshold settings
#define LPMS_DISABLE_GYR_THRESHOLD 	0x00000000
#define LPMS_ENABLE_GYR_THRESHOLD 	0x00000001

// Magnetometer autocalibration settings
#define LPMS_ENABLE_MAG_AUTOCAL 	0x00000001
#define LPMS_DISABLE_MAG_AUTOCAL 	0x00000000

// Gyroscope automcalibration settings
#define LPMS_ENABLE_GYR_AUTOCAL 	0x00000001
#define LPMS_DISABLE_GYR_AUTOCAL 	0x00000000

// CAN bus stream formats
#define LPMS_SET_STREAM_CAN_LPBUS 	0x00000000
#define LPMS_SET_STREAM_CAN_CUSTOM1 	0x00000001
#define LPMS_SET_STREAM_CAN_OPEN 	0x00000002
#define LPMS_SET_STREAM_CAN_CUSTOM2 	0x00000003
#define LPMS_SET_STREAM_CAN_CUSTOM3 	0x00000004

// LpFilter modes
#define LPMS_FILTER_GYR 		0x00000000
#define LPMS_FILTER_GYR_ACC 		0x00000001
#define LPMS_FILTER_GYR_ACC_MAG 	0x00000002
#define LPMS_FILTER_ACC_MAG 		0x00000003
#define LPMS_FILTER_GYR_ACC_EULER 	0x00000004

// Self-test settings
#define LPMS_SELF_TEST_OFF 	0x00000000
#define LPMS_SELF_TEST_ON 	0x00000001

// Low-pass filter settings
#define LPMS_LP_OFF 	0x00000000
#define LPMS_LP_01 	0x00000001
#define LPMS_LP_005 	0x00000002
#define LPMS_LP_001 	0x00000003
#define LPMS_LP_0005 	0x00000004
#define LPMS_LP_0001 	0x00000005

// CAN heartbeat settings
#define LPMS_CAN_HEARTBEAT_005			0x00000000
#define LPMS_CAN_HEARTBEAT_010			0x00000001
#define LPMS_CAN_HEARTBEAT_020			0x00000002
#define LPMS_CAN_HEARTBEAT_050			0x00000003
#define LPMS_CAN_HEARTBEAT_100			0x00000004

// Linear acceleration compensation mode settings
#define LPMS_LIN_ACC_COMP_MODE_OFF		0x00000000
#define LPMS_LIN_ACC_COMP_MODE_WEAK		0x00000001
#define LPMS_LIN_ACC_COMP_MODE_MEDIUM		0x00000002
#define LPMS_LIN_ACC_COMP_MODE_STRONG		0x00000003
#define LPMS_LIN_ACC_COMP_MODE_ULTRA		0x00000004

// Rotation acceleration compensation settings
#define LPMS_CENTRI_COMP_MODE_OFF		0x00000000
#define LPMS_CENTRI_COMP_MODE_ON		0x00000001

// CAN bus message order settings
#define LPMS_CAN_SEQUENTIAL_MODE (uint32_t)(0x00000001 << 0)
#define LPMS_CAN_FIXEDPOINT_MODE (uint32_t)(0x00000001 << 1)

#define LPMS_LPBUS_DATA_MODE_32 0x0
#define LPMS_LPBUS_DATA_MODE_16 0x1

#define LPMS_OFFSET_MODE_OBJECT 0x0
#define LPMS_OFFSET_MODE_HEADING 0x1
#define LPMS_OFFSET_MODE_ALIGNMENT 0x2

// Register data type
typedef struct _LpmsReg {
	uint32_t data[REG_ARRAY_SIZE];
} LpmsReg;

// Writes a portion of RAM to flash memory
uint8_t writeRamToFlash(__IO uint32_t* flashAddress, uint32_t* ramBuffer, uint32_t length);

// Loads a portion of flash memory into RAM
void loadFlashToRam(__IO uint32_t* flashAddress, uint32_t* ramBuffer, uint32_t length);

// Resets to default settings
uint8_t resetToFactory(void);

// Copies flash memory to RAM
void copyFlashToRam_256bytes(__IO uint32_t* flashAddress, uint32_t* ramBuffer);

// Copies RAM to flash
uint8_t copyRamToFlash_256bytes(__IO uint32_t* flashAddress, uint32_t* ramBuffer);

uint8_t copyRamToFlash_128bytes(__IO uint32_t* flashAddress, uint32_t* ramBuffer);

// Writes complete register set
uint8_t writeCompleteRegisterSet(void);

// Erases complete register set
uint8_t eraseCompleteRegisterSet(void);

#endif