/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_REGISTER_DEFINITIONS
#define LPMS_REGISTER_DEFINITIONS

/* This file contains the low level register definitions for LPMS devices. */

#define COMMAND_START_ADDRESS 	0

// Acknowledged and Not-acknowledged identifier
#define REPLY_ACK 				(COMMAND_START_ADDRESS + 0)
#define REPLY_NACK 				(COMMAND_START_ADDRESS + 1)

// Firmware update and in-application-programmer upload
#define UPDATE_FIRMWARE 		(COMMAND_START_ADDRESS + 2)	
#define UPDATE_IAP 				(COMMAND_START_ADDRESS + 3)	

// Configuration and status
#define GET_CONFIG 				(COMMAND_START_ADDRESS + 4)	
#define GET_STATUS 				(COMMAND_START_ADDRESS + 5)	

// Mode switching
#define GOTO_COMMAND_MODE 		(COMMAND_START_ADDRESS + 6)	
#define GOTO_STREAM_MODE 		(COMMAND_START_ADDRESS + 7)	
#define GOTO_SLEEP_MODE 		(COMMAND_START_ADDRESS + 8)	

// Data transmission
#define GET_SENSOR_DATA 		(COMMAND_START_ADDRESS + 9)
#define SET_TRANSMIT_DATA 		(COMMAND_START_ADDRESS + 10)
#define SET_STREAM_FREQ 		(COMMAND_START_ADDRESS + 11)	
#define GET_ROLL 				(COMMAND_START_ADDRESS + 12)
#define GET_PITCH 				(COMMAND_START_ADDRESS + 13)
#define GET_YAW 				(COMMAND_START_ADDRESS + 14)

// Register value save and reset
#define WRITE_REGISTERS 		(COMMAND_START_ADDRESS + 15)
#define RESTORE_FACTORY_VALUE 	(COMMAND_START_ADDRESS + 16)	

// Reference setting and offset reset 
#define RESET_REFERENCE 			(COMMAND_START_ADDRESS + 17)	
#define SET_ORIENTATION_OFFSET		(COMMAND_START_ADDRESS + 18)	

// Self-test
#define SELF_TEST 				(COMMAND_START_ADDRESS + 19)	

// IMU ID setting
#define SET_IMU_ID 				(COMMAND_START_ADDRESS + 20)	
#define GET_IMU_ID 				(COMMAND_START_ADDRESS + 21)	

// Gyroscope settings
#define START_GYR_CALIBRA 		(COMMAND_START_ADDRESS + 22)	
#define ENABLE_GYR_AUTOCAL 		(COMMAND_START_ADDRESS + 23)	
#define ENABLE_GYR_THRES 		(COMMAND_START_ADDRESS + 24)	
#define SET_GYR_RANGE 			(COMMAND_START_ADDRESS + 25)	
#define GET_GYR_RANGE 			(COMMAND_START_ADDRESS + 26)	

// Accelerometer settings
#define SET_ACC_BIAS 			(COMMAND_START_ADDRESS + 27)	
#define GET_ACC_BIAS 			(COMMAND_START_ADDRESS + 28)	
#define SET_ACC_ALIGN_MATRIX	(COMMAND_START_ADDRESS + 29)	
#define GET_ACC_ALIGN_MATRIX 	(COMMAND_START_ADDRESS + 30)	
#define SET_ACC_RANGE 			(COMMAND_START_ADDRESS + 31)	
#define GET_ACC_RANGE 			(COMMAND_START_ADDRESS + 32)	

// Magnetometer settings
#define SET_MAG_RANGE 				(COMMAND_START_ADDRESS + 33)	
#define GET_MAG_RANGE 				(COMMAND_START_ADDRESS + 34)	
#define SET_HARD_IRON_OFFSET 		(COMMAND_START_ADDRESS + 35)	
#define GET_HARD_IRON_OFFSET 		(COMMAND_START_ADDRESS + 36)	
#define SET_SOFT_IRON_MATRIX 		(COMMAND_START_ADDRESS + 37)	
#define GET_SOFT_IRON_MATRIX 		(COMMAND_START_ADDRESS + 38)
#define SET_FIELD_ESTIMATE 			(COMMAND_START_ADDRESS + 39)	
#define GET_FIELD_ESTIMATE 			(COMMAND_START_ADDRESS + 40)

// Filter settings
#define SET_FILTER_MODE 			(COMMAND_START_ADDRESS + 41)	
#define GET_FILTER_MODE 			(COMMAND_START_ADDRESS + 42)	
#define SET_FILTER_PRESET 			(COMMAND_START_ADDRESS + 43)	
#define GET_FILTER_PRESET 			(COMMAND_START_ADDRESS + 44)	

// CAN settings
#define SET_CAN_STREAM_FORMAT 		(COMMAND_START_ADDRESS + 45)	
#define SET_CAN_BAUDRATE 			(COMMAND_START_ADDRESS + 46)

// Additional settings
#define GET_FIRMWARE_VERSION		(COMMAND_START_ADDRESS + 47)

#define SET_GYR_ALIGN_BIAS 			(COMMAND_START_ADDRESS + 48)	
#define GET_GYR_ALIGN_BIAS 			(COMMAND_START_ADDRESS + 49)	

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

#define SET_RAW_DATA_LP 			(COMMAND_START_ADDRESS + 60)
#define GET_RAW_DATA_LP 			(COMMAND_START_ADDRESS + 61)	

#define SET_CAN_MAPPING 			(COMMAND_START_ADDRESS + 62)
#define GET_CAN_MAPPING 			(COMMAND_START_ADDRESS + 63)

#define SET_CAN_HEARTBEAT 			(COMMAND_START_ADDRESS + 64)
#define GET_CAN_HEARTBEAT 			(COMMAND_START_ADDRESS + 65)

#define RESET_TIMESTAMP 			(COMMAND_START_ADDRESS + 66)

#define SET_LIN_ACC_COMP_MODE		(COMMAND_START_ADDRESS + 67)
#define GET_LIN_ACC_COMP_MODE		(COMMAND_START_ADDRESS + 68)

#define SET_CENTRI_COMP_MODE		(COMMAND_START_ADDRESS + 69)
#define GET_CENTRI_COMP_MODE		(COMMAND_START_ADDRESS + 70)

#define GET_CAN_CONFIGURATION		(COMMAND_START_ADDRESS + 71)
#define SET_CAN_CHANNEL_MODE 		(COMMAND_START_ADDRESS + 72)
#define SET_CAN_POINT_MODE 			(COMMAND_START_ADDRESS + 73)
#define SET_CAN_START_ID 			(COMMAND_START_ADDRESS + 74)
#define SET_LPBUS_DATA_MODE			(COMMAND_START_ADDRESS + 75)

#define SET_MAG_ALIGNMENT_MATRIX	(COMMAND_START_ADDRESS + 76)
#define SET_MAG_ALIGNMENT_BIAS		(COMMAND_START_ADDRESS + 77)
#define SET_MAG_REFRENCE			(COMMAND_START_ADDRESS + 78)
#define GET_MAG_ALIGNMENT_MATRIX	(COMMAND_START_ADDRESS + 79)
#define GET_MAG_ALIGNMENT_BIAS		(COMMAND_START_ADDRESS + 80)
#define GET_MAG_REFERENCE			(COMMAND_START_ADDRESS + 81)
#define RESET_ORIENTATION_OFFSET	(COMMAND_START_ADDRESS + 82)

#define SET_UART_BAUDRATE			(COMMAND_START_ADDRESS + 83)
#define GET_UART_BAUDRATE			(COMMAND_START_ADDRESS + 84)

#define	IAP_FLASH_START_ADDRESS 			(uint32_t)0x08060000
#define IAP_MAX_PACKET_SIZE 				64
#define USER_APPLICATION_BACKUP_ADDRESS 	(uint32_t)0x08020000
#define USER_APPLICATION_MAX_PACKET_SIZE 	512

#define	USER_FLASH_START_ADDRESS 		(uint32_t)0x08040000
#define	FACTORY_FLASH_START_ADDRESS 	(uint32_t)0x08050000
#define CHECK_USER_FLASH() 				((uint32_t)( *(__IO u32*)(USER_FLASH_START_ADDRESS) ) != 0xFFFFFFFF)
#define CHECK_FACTORY_FLASH() 			((uint32_t)( *(__IO u32*)(FACTORY_FLASH_START_ADDRESS) ) == 0xFFFFFFFF)

#define NOT_USED_0								(uint32_t)(0x00000001 << 31)
#define LPMS_GYR_AUTOCAL_ENABLED 				(uint32_t)(0x00000001 << 30)
#define NOT_USED_1				 				(uint32_t)(0x00000001 << 29)
#define NOT_USED_2			 					(uint32_t)(0x00000001 << 28)
#define LPMS_GAIT_TRACKING_ENABLED 				(uint32_t)(0x00000001 << 27)
#define LPMS_HEAVEMOTION_ENABLED 				(uint32_t)(0x00000001 << 26)
#define	LPMS_ACC_COMP_ENABLED 					(uint32_t)(0x00000001 << 25)
#define	LPMS_MAG_COMP_ENABLED 					(uint32_t)(0x00000001 << 24)
#define	LPMS_GYR_THRES_ENABLED 					(uint32_t)(0x00000001 << 23)
#define	LPMS_LPBUS_DATA_MODE_16BIT_ENABLED		(uint32_t)(0x00000001 << 22)
#define	LPMS_LINACC_OUTPUT_ENABLED 				(uint32_t)(0x00000001 << 21)
#define LPMS_DYNAMIC_COVAR_ENABLED 				(uint32_t)(0x00000001 << 20)
#define LPMS_ALTITUDE_OUTPUT_ENABLED 			(uint32_t)(0x00000001 << 19)
#define	LPMS_QUAT_OUTPUT_ENABLED 				(uint32_t)(0x00000001 << 18)
#define	LPMS_EULER_OUTPUT_ENABLED 				(uint32_t)(0x00000001 << 17)
#define	LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED 	(uint32_t)(0x00000001 << 16)
#define	LPMS_GYR_CALIBRA_ENABLED 				(uint32_t)(0x00000001 << 15)
#define	LPMS_HEAVEMOTION_OUTPUT_ENABLED 		(uint32_t)(0x00000001 << 14)
#define	LPMS_TEMPERATURE_OUTPUT_ENABLED			(uint32_t)(0x00000001 << 13)
#define	LPMS_GYR_RAW_OUTPUT_ENABLED				(uint32_t)(0x00000001 << 12)
#define	LPMS_ACC_RAW_OUTPUT_ENABLED				(uint32_t)(0x00000001 << 11)
#define	LPMS_MAG_RAW_OUTPUT_ENABLED				(uint32_t)(0x00000001 << 10)
#define	LPMS_PRESSURE_OUTPUT_ENABLED			(uint32_t)(0x00000001 << 9)

#define GYR_RANGE_250DPS				250
#define GYR_RANGE_500DPS				500
#define GYR_RANGE_2000DPS				2000

#define GYR_GAIN_250DPS					0.00875
#define GYR_GAIN_500DPS					0.0175
#define GYR_GAIN_2000DPS				0.070

#define GYR_OUTPUT_DATA_RATE_95HZ	95
#define GYR_OUTPUT_DATA_RATE_190HZ 190
#define GYR_OUTPUT_DATA_RATE_380HZ 380
#define GYR_OUTPUT_DATA_RATE_760HZ	760

#define ACC_RANGE_2G					2
#define ACC_RANGE_4G					4
#define ACC_RANGE_8G					8
#define ACC_RANGE_16G					16

#define ACC_GAIN_2G						(1.0f / 16384.0f)
#define ACC_GAIN_4G						(1.0f / 8192.0f)
#define ACC_GAIN_8G						(1.0f / 4096.0f)
#define ACC_GAIN_16G 					(1.0f / 2048.0f)

#define ACC_OUTPUT_DATA_RATE_0HZ		0
#define ACC_OUTPUT_DATA_RATE_1HZ		1
#define ACC_OUTPUT_DATA_RATE_10HZ		10
#define ACC_OUTPUT_DATA_RATE_25HZ		25
#define ACC_OUTPUT_DATA_RATE_50HZ		50
#define ACC_OUTPUT_DATA_RATE_100HZ		100
#define ACC_OUTPUT_DATA_RATE_200HZ		200
#define ACC_OUTPUT_DATA_RATE_400HZ		400
#define ACC_OUTPUT_DATA_RATE_1344HZ		1344

#define ACC_NORMAL_POWER_MODE			0
#define ACC_LOW_POWER_MODE				1

#define MAG_RANGE_130UT			130
#define MAG_RANGE_190UT			190
#define MAG_RANGE_250UT			250
#define MAG_RANGE_400UT			400
#define MAG_RANGE_470UT			470
#define MAG_RANGE_560UT			560
#define MAG_RANGE_810UT			810

#define MAG_GAIN_XY_130UT		(100.0f / 1100.0f)
#define MAG_GAIN_Z_130UT		(100.0f / 980.0f) 
#define MAG_GAIN_XY_190UT		(100.0f / 855.0f)
#define MAG_GAIN_Z_190UT		(100.0f / 760.0f)
#define MAG_GAIN_XY_250UT		(100.0f / 670.0f)
#define MAG_GAIN_Z_250UT		(100.0f / 600.0f)
#define MAG_GAIN_XY_400UT		(100.0f / 450.0f)
#define MAG_GAIN_Z_400UT		(100.0f / 400.0f)
#define MAG_GAIN_XY_470UT		(100.0f / 400.0f)
#define MAG_GAIN_Z_470UT		(100.0f / 355.0f)
#define MAG_GAIN_XY_560UT		(100.0f / 330.0f)
#define MAG_GAIN_Z_560UT		(100.0f / 295.0f)
#define MAG_GAIN_XY_810UT		(100.0f / 230.0f)
#define MAG_GAIN_Z_810UT		(100.0f / 205.0f)

#define MAG_OUTPUT_DATA_RATE_1HZ			1
#define MAG_OUTPUT_DATA_RATE_2HZ			2
#define MAG_OUTPUT_DATA_RATE_3HZ			3
#define MAG_OUTPUT_DATA_RATE_8HZ			8
#define MAG_OUTPUT_DATA_RATE_15HZ			15
#define MAG_OUTPUT_DATA_RATE_30HZ			30
#define MAG_OUTPUT_DATA_RATE_75HZ			75
#define MAG_OUTPUT_DATA_RATE_220HZ			220

#define MAG_NORMAL_POWER_MODE				0
#define MAG_SLEEP_POWER_MODE				1

#define LPMS_STREAM_FREQ_5HZ_ENABLED 		0x00000000
#define LPMS_STREAM_FREQ_10HZ_ENABLED 		0x00000001
#define LPMS_STREAM_FREQ_30HZ_ENABLED 		0x00000002
#define LPMS_STREAM_FREQ_50HZ_ENABLED 		0x00000003
#define LPMS_STREAM_FREQ_100HZ_ENABLED 		0x00000004
#define LPMS_STREAM_FREQ_200HZ_ENABLED 		0x00000005
#define LPMS_STREAM_FREQ_500HZ_ENABLED 		0x00000006
#define LPMS_STREAM_FREQ_1000HZ_ENABLED		0x00000007
#define LPMS_STREAM_FREQ_MASK 				0x00000007

#define LPMS_STREAM_FREQ_5HZ 				(uint32_t)5
#define LPMS_STREAM_FREQ_10HZ 				(uint32_t)10
#define LPMS_STREAM_FREQ_30HZ 				(uint32_t)30
#define LPMS_STREAM_FREQ_50HZ 				(uint32_t)50
#define LPMS_STREAM_FREQ_100HZ 				(uint32_t)100
#define LPMS_STREAM_FREQ_200HZ 				(uint32_t)200
#define LPMS_STREAM_FREQ_500HZ 				(uint32_t)500
#define LPMS_STREAM_FREQ_1000HZ				(uint32_t)1000

#define LPMS_BT_BAUDRATE_9600_ENABLED 		0x00000000
#define LPMS_BT_BAUDRATE_19200_ENABLED 		0x00000008
#define LPMS_BT_BAUDRATE_38400_ENABLED 		0x00000010
#define LPMS_BT_BAUDRATE_57600_ENABLED 		0x00000018
#define LPMS_BT_BAUDRATE_115200_ENABLED 	0x00000020
#define LPMS_BT_BAUDRATE_230400_ENABLED 	0x00000028
#define LPMS_BT_BAUDRATE_460800_ENABLED 	0x00000030
#define LPMS_BT_BAUDRATE_921600_ENABLED 	0x00000038
#define LPMS_CANBUS_BAUDRATE_10K_ENABLED	0x00000000
#define LPMS_CANBUS_BAUDRATE_20K_ENABLED 	0x00000008
#define LPMS_CANBUS_BAUDRATE_50K_ENABLED 	0x00000010
#define LPMS_CANBUS_BAUDRATE_125K_ENABLED 	0x00000018
#define LPMS_CANBUS_BAUDRATE_250K_ENABLED 	0x00000020
#define LPMS_CANBUS_BAUDRATE_500K_ENABLED 	0x00000028
#define LPMS_CANBUS_BAUDRATE_800K_ENABLED 	0x00000030
#define LPMS_CANBUS_BAUDRATE_1M_ENABLED 	0x00000038

#define LPMS_BAUDRATE_MASK 				0x00000038
#define LPMS_CAN_BAUDRATE_MASK 			0x00000038

#define LPMS_BT_BAUDRATE_9600 			(uint32_t)9600
#define LPMS_BT_BAUDRATE_19200 			(uint32_t)19200
#define LPMS_BT_BAUDRATE_38400 			(uint32_t)38400
#define LPMS_BT_BAUDRATE_57600 			(uint32_t)57600
#define LPMS_BT_BAUDRATE_115200 		(uint32_t)115200
#define LPMS_BT_BAUDRATE_230400 		(uint32_t)230400
#define LPMS_BT_BAUDRATE_460800 		(uint32_t)460800
#define LPMS_BT_BAUDRATE_921600 		(uint32_t)921600
#define LPMS_CANBUS_BAUDRATE_10K 		(uint32_t)10000
#define LPMS_CANBUS_BAUDRATE_20K 		(uint32_t)20000
#define LPMS_CANBUS_BAUDRATE_50K 		(uint32_t)50000
#define LPMS_CANBUS_BAUDRATE_125K 		(uint32_t)125000
#define LPMS_CANBUS_BAUDRATE_250K 		(uint32_t)250000
#define LPMS_CANBUS_BAUDRATE_500K 		(uint32_t)500000
#define LPMS_CANBUS_BAUDRATE_800K 		(uint32_t)800000
#define LPMS_CANBUS_BAUDRATE_1M 		(uint32_t)1000000

#define LPMS_FILTER_PRM_SET_1 			0x00000000
#define LPMS_FILTER_PRM_SET_2 			0x00000001
#define LPMS_FILTER_PRM_SET_3 			0x00000002
#define LPMS_FILTER_PRM_SET_4 			0x00000003

#define LPMS_ENABLE_GYR_THRESHOLD 		0x00000001
#define LPMS_DISABLE_GYR_THRESHOLD 		0x00000000

#define LPMS_ENABLE_MAG_AUTOCAL 		0x00000001
#define LPMS_DISABLE_MAG_AUTOCAL 		0x00000000

#define LPMS_ENABLE_GYR_AUTOCAL 		0x00000001
#define LPMS_DISABLE_GYR_AUTOCAL 		0x00000000

/* #define LPMS_HEAVEMOTION_ENABLED 		0x00000001
#define LPMS_HEAVEMOTION_DISABLED		0x00000000 */

#define LPMS_FILTER_GYR 				0x00000000
#define LPMS_FILTER_GYR_ACC 			0x00000001
#define LPMS_FILTER_GYR_ACC_MAG 		0x00000002
#define LPMS_FILTER_ACC_MAG 			0x00000003
#define LPMS_FILTER_GYR_ACC_EULER 		0x00000004

#define LPMS_SET_STREAM_CAN_LPBUS 		0x00000000
#define LPMS_SET_STREAM_CAN_CUSTOM1 	0x00000001
#define LPMS_SET_STREAM_CAN_OPEN 		0x00000002
#define LPMS_SET_STREAM_CAN_CUSTOM2 	0x00000003

#define LPMS_LP_OFF 					0x00000000
#define LPMS_LP_01 						0x00000001
#define LPMS_LP_005 					0x00000002
#define LPMS_LP_001 					0x00000003
#define LPMS_LP_0005 					0x00000004
#define LPMS_LP_0001 					0x00000005

#define LPMS_CAN_HEARTBEAT_005			0x00000000
#define LPMS_CAN_HEARTBEAT_010			0x00000001
#define LPMS_CAN_HEARTBEAT_020			0x00000002
#define LPMS_CAN_HEARTBEAT_050			0x00000003
#define LPMS_CAN_HEARTBEAT_100			0x00000004

#define LPMS_COMMAND_MODE				(0x0001 << 0)
#define LPMS_STREAM_MODE				(0x0001 << 1)
#define LPMS_SLEEP_MODE					(0x0001 << 2)
#define LPMS_GYR_CALIBRATION_RUNNING	(0x0001 << 3)
#define LPMS_MAG_CALIBRATION_RUNNING	(0x0001 << 4)
#define	LPMS_GYR_INIT_FAILED			(0x0001 << 5)
#define	LPMS_ACC_INIT_FAILED			(0x0001 << 6)
#define	LPMS_MAG_INIT_FAILED			(0x0001 << 7)
#define LPMS_PRESSURE_INIT_FAILED		(0x0001 << 8)
#define	LPMS_GYR_UNRESPONSIVE			(0x0001 << 9)
#define	LPMS_ACC_UNRESPONSIVE			(0x0001 << 10)
#define	LPMS_MAG_UNRESPONSIVE			(0x0001 << 11)
#define	LPMS_FLASH_WRITE_FAILED			(0x0001 << 12)
#define LPMS_SET_BAUDRATE_FAILED		(0x0001 << 13)
#define LPMS_SET_BROADCAST_FREQ_FAILED	(0x0001 << 14)
#define LPMS_REF_CALIBRATION_RUNNING 	(0x0001 << 15)

#define LPMS_LIN_ACC_COMP_MODE_OFF			0x00000000
#define LPMS_LIN_ACC_COMP_MODE_WEAK			0x00000001
#define LPMS_LIN_ACC_COMP_MODE_MEDIUM		0x00000002
#define LPMS_LIN_ACC_COMP_MODE_STRONG		0x00000003
#define LPMS_LIN_ACC_COMP_MODE_ULTRA		0x00000004

#define LPMS_CENTRI_COMP_MODE_OFF			0x00000000
#define LPMS_CENTRI_COMP_MODE_ON			0x00000001

#define LPMS_CAN_SEQUENTIAL_MODE (uint32_t)(0x00000001 << 0)
#define LPMS_CAN_FIXEDPOINT_MODE (uint32_t)(0x00000001 << 1)

#define LPMS_LPBUS_DATA_MODE_32 0x0
#define LPMS_LPBUS_DATA_MODE_16 0x1

#define LPMS_UART_BAUDRATE_19200		0x00000000
#define LPMS_UART_BAUDRATE_57600		0x00000001
#define LPMS_UART_BAUDRATE_115200		0x00000002
#define LPMS_UART_BAUDRATE_921600		0x00000003

#endif