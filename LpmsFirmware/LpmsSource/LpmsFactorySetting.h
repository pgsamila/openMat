/***********************************************************************
** Sensor factory settings
**
** Copyright (C) 2013 LP-RESEARCH Inc.
** All rights reserved.
** Contact: info@lp-research.com
***********************************************************************/

#ifndef LPMS_FACTORY_SETTING_H
#define LPMS_FACTORY_SETTING_H

#include "LpmsL3gd20.h"
#include "LpmsLsm303dlhc.h"

// Compile time switches -->

/* IMPORTANT: LPMS type switches now selected through compiler command line.
	Use Project->Edit Configurations to change or batch build */

// Enabled Bluetooth Low Energy Optimizations based on TTL LPMS-CU
// #define LPMS_BLE

// Enables CAN bus interface
// #define USE_CANBUS_INTERFACE

// Enables NEW LPMS-CU. 
// Note: USE_CANBUS_INTERFACE and USE_LPMSCU_NEW must both be selected.
// #define USE_LPMSCU_NEW

// Enables RS-232 (New LPMS-CU only!)
// #define USE_RS232_INTERFACE

// Enables TTL UART (New LPMS-CU only!)
// #define USE_TTL_UART_INTERFACE

// Enables Bluetooth interface
// #define USE_BLUETOOTH_INTERFACE

// Enables heave motion tracking
// #define USE_HEAVEMOTION

// <-- Deprecated type switched until here

// Enables watchdog timer
// #define ENABLE_WATCHDOG

// To enable pressure sensing
#define ENABLE_PRESSURE

// Enable Bluetooth low-latency
#define ENABLE_LOWLATENCY

// <--

// Firmware version information
#define FIRMWARE_VERSION_DIGIT0 0
#define FIRMWARE_VERSION_DIGIT1 3
#define FIRMWARE_VERSION_DIGIT2 1

// Interface dependent factory settings
#ifdef USE_BLUETOOTH_INTERFACE                    
	#ifdef USE_HEAVEMOTION
		#define	LPMS_FACTORY_CONFIG	(LPMS_ACC_COMP_ENABLED | \
				LPMS_GYR_RAW_OUTPUT_ENABLED | \
				LPMS_ACC_RAW_OUTPUT_ENABLED | \
				LPMS_MAG_RAW_OUTPUT_ENABLED | \
				LPMS_QUAT_OUTPUT_ENABLED | \
				LPMS_EULER_OUTPUT_ENABLED | \
				LPMS_STREAM_FREQ_100HZ_ENABLED | \
				LPMS_BT_BAUDRATE_230400_ENABLED | \
				LPMS_HEAVEMOTION_ENABLED | \
				LPMS_HEAVEMOTION_OUTPUT_ENABLED | \
				LPMS_LINACC_OUTPUT_ENABLED | \
				LPMS_GYR_AUTOCAL_ENABLED)
	#elif USE_GAIT_TRACKING
		#define	LPMS_FACTORY_CONFIG	(LPMS_ACC_COMP_ENABLED | \
				LPMS_GYR_RAW_OUTPUT_ENABLED | \
				LPMS_ACC_RAW_OUTPUT_ENABLED | \
				LPMS_MAG_RAW_OUTPUT_ENABLED | \
				LPMS_QUAT_OUTPUT_ENABLED | \
				LPMS_EULER_OUTPUT_ENABLED | \
				LPMS_STREAM_FREQ_100HZ_ENABLED | \
				LPMS_BT_BAUDRATE_230400_ENABLED | \
				LPMS_LINACC_OUTPUT_ENABLED | \
				LPMS_GAIT_TRACKING_ENABLED | \
				LPMS_GYR_AUTOCAL_ENABLED)
	#else
		#define	LPMS_FACTORY_CONFIG	(LPMS_ACC_COMP_ENABLED | \
				LPMS_GYR_RAW_OUTPUT_ENABLED | \
				LPMS_ACC_RAW_OUTPUT_ENABLED | \
				LPMS_MAG_RAW_OUTPUT_ENABLED | \
				LPMS_QUAT_OUTPUT_ENABLED | \
				LPMS_EULER_OUTPUT_ENABLED | \
				LPMS_STREAM_FREQ_100HZ_ENABLED | \
				LPMS_BT_BAUDRATE_230400_ENABLED | \
				LPMS_LINACC_OUTPUT_ENABLED | \
				LPMS_GYR_AUTOCAL_ENABLED)
	#endif
#endif

#ifdef USE_CANBUS_INTERFACE
	#ifdef LPMS_BLE
		#define	LPMS_FACTORY_CONFIG	(LPMS_ACC_COMP_ENABLED | \
				LPMS_QUAT_OUTPUT_ENABLED | \
				LPMS_STREAM_FREQ_30HZ_ENABLED | \
				LPMS_CAN_BAUDRATE_1M_ENABLED | \
				LPMS_HEAVEMOTION_ENABLED | \
				LPMS_HEAVEMOTION_OUTPUT_ENABLED | \
				LPMS_GYR_AUTOCAL_ENABLED | \
				LPMS_LPBUS_DATA_MODE_16BIT_ENABLED)
	#else   
		#ifdef USE_HEAVEMOTION
			#define	LPMS_FACTORY_CONFIG	(LPMS_ACC_COMP_ENABLED | \
					LPMS_GYR_RAW_OUTPUT_ENABLED | \
					LPMS_ACC_RAW_OUTPUT_ENABLED | \
					LPMS_MAG_RAW_OUTPUT_ENABLED | \
					LPMS_QUAT_OUTPUT_ENABLED | \
					LPMS_EULER_OUTPUT_ENABLED | \
					LPMS_STREAM_FREQ_100HZ_ENABLED | \
					LPMS_CAN_BAUDRATE_1M_ENABLED | \
					LPMS_HEAVEMOTION_ENABLED | \
					LPMS_HEAVEMOTION_OUTPUT_ENABLED | \
					LPMS_LINACC_OUTPUT_ENABLED | \
					LPMS_GYR_AUTOCAL_ENABLED)
		#else
			#define	LPMS_FACTORY_CONFIG	(LPMS_ACC_COMP_ENABLED | \
					LPMS_GYR_RAW_OUTPUT_ENABLED | \
					LPMS_ACC_RAW_OUTPUT_ENABLED | \
					LPMS_MAG_RAW_OUTPUT_ENABLED | \
					LPMS_QUAT_OUTPUT_ENABLED | \
					LPMS_EULER_OUTPUT_ENABLED | \
					LPMS_STREAM_FREQ_100HZ_ENABLED | \
					LPMS_CAN_BAUDRATE_1M_ENABLED | \
					LPMS_LINACC_OUTPUT_ENABLED | \
					LPMS_GYR_AUTOCAL_ENABLED)
		#endif
	#endif
#endif

// Factory settings
#define LPMS_FACTORY_IMU_ID			0
#define LPMS_FACTORY_GYR_RANGE			GYR_RANGE_2000DPS
#define LPMS_FACTORY_ACC_RANGE			ACC_RANGE_4G
#define LPMS_FACTORY_MAG_RANGE			MAG_RANGE_250UT
#define LPMS_FACTORY_GYR_OUTPUT_RATE		GYR_OUTPUT_DATA_RATE_760HZ
#define LPMS_FACTORY_ACC_OUTPUT_RATE		ACC_OUTPUT_DATA_RATE_1344HZ
#define LPMS_FACTORY_MAG_OUTPUT_RATE		MAG_OUTPUT_DATA_RATE_220HZ

#define	LPMS_FACTORY_GYR_BIAS_X			0.0f
#define	LPMS_FACTORY_GYR_BIAS_Y			0.0f
#define	LPMS_FACTORY_GYR_BIAS_Z			0.0f

#define	LPMS_FACTORY_GYR_GAIN_X			GYR_GAIN_2000DPS
#define	LPMS_FACTORY_GYR_GAIN_Y			GYR_GAIN_2000DPS
#define	LPMS_FACTORY_GYR_GAIN_Z			GYR_GAIN_2000DPS

#define	LPMS_FACTORY_ACC_BIAS_X			0.0
#define	LPMS_FACTORY_ACC_BIAS_Y			0.0
#define	LPMS_FACTORY_ACC_BIAS_Z			0.0

#define	LPMS_FACTORY_ACC_GAIN_X			ACC_GAIN_4G
#define	LPMS_FACTORY_ACC_GAIN_Y			ACC_GAIN_4G
#define	LPMS_FACTORY_ACC_GAIN_Z			ACC_GAIN_4G

#define	LPMS_FACTORY_MAG_BIAS_X			0.0f
#define	LPMS_FACTORY_MAG_BIAS_Y			0.0f
#define	LPMS_FACTORY_MAG_BIAS_Z			0.0f

#define	LPMS_FACTORY_MAG_GAIN_X			MAG_GAIN_XY_250UT
#define	LPMS_FACTORY_MAG_GAIN_Y			MAG_GAIN_XY_250UT
#define	LPMS_FACTORY_MAG_GAIN_Z			MAG_GAIN_Z_250UT

#define	LPMS_FACTORY_GYR_ALIG_00		1.0f
#define	LPMS_FACTORY_GYR_ALIG_01		0.0f
#define	LPMS_FACTORY_GYR_ALIG_02		0.0f
#define	LPMS_FACTORY_GYR_ALIG_10		0.0f
#define	LPMS_FACTORY_GYR_ALIG_11		1.0f
#define	LPMS_FACTORY_GYR_ALIG_12		0.0f
#define	LPMS_FACTORY_GYR_ALIG_20		0.0f
#define	LPMS_FACTORY_GYR_ALIG_21		0.0f
#define	LPMS_FACTORY_GYR_ALIG_22		1.0f

#define	LPMS_FACTORY_ACC_ALIG_00		1.0f
#define	LPMS_FACTORY_ACC_ALIG_01		0.0f
#define	LPMS_FACTORY_ACC_ALIG_02		0.0f
#define	LPMS_FACTORY_ACC_ALIG_10		0.0f
#define	LPMS_FACTORY_ACC_ALIG_11		1.0f
#define	LPMS_FACTORY_ACC_ALIG_12		0.0f
#define	LPMS_FACTORY_ACC_ALIG_20		0.0f
#define	LPMS_FACTORY_ACC_ALIG_21		0.0f
#define	LPMS_FACTORY_ACC_ALIG_22		1.0f

#define	LPMS_FACTORY_MAG_ALIG_00		1.0f
#define	LPMS_FACTORY_MAG_ALIG_01		0.0f
#define	LPMS_FACTORY_MAG_ALIG_02		0.0f
#define	LPMS_FACTORY_MAG_ALIG_10		0.0f
#define	LPMS_FACTORY_MAG_ALIG_11		1.0f
#define	LPMS_FACTORY_MAG_ALIG_12		0.0f
#define	LPMS_FACTORY_MAG_ALIG_20		0.0f
#define	LPMS_FACTORY_MAG_ALIG_21		0.0f
#define	LPMS_FACTORY_MAG_ALIG_22		1.0f

#define	LPMS_FACTORY_MAG_SOFT_00		1.0f
#define	LPMS_FACTORY_MAG_SOFT_01		0.0f
#define	LPMS_FACTORY_MAG_SOFT_02		0.0f
#define	LPMS_FACTORY_MAG_SOFT_10		0.0f
#define	LPMS_FACTORY_MAG_SOFT_11		1.0f
#define	LPMS_FACTORY_MAG_SOFT_12		0.0f
#define	LPMS_FACTORY_MAG_SOFT_20		0.0f
#define	LPMS_FACTORY_MAG_SOFT_21		0.0f
#define	LPMS_FACTORY_MAG_SOFT_22		1.0f

#define	LPMS_FACTORY_ACC_REF_X			0.0f
#define	LPMS_FACTORY_ACC_REF_Y			0.0f
#define	LPMS_FACTORY_ACC_REF_Z			-1.0f

#define	LPMS_FACTORY_MAG_REF_X			0.0f
#define	LPMS_FACTORY_MAG_REF_Y			1.0f
#define	LPMS_FACTORY_MAG_REF_Z			0.0f

#define LPMS_FACTORY_GYR_THRES_X		0.25f
#define LPMS_FACTORY_GYR_THRES_Y		0.25f
#define LPMS_FACTORY_GYR_THRES_Z		0.25f

#define LPMS_FACTORY_MAG_THRES_X		100.0f
#define LPMS_FACTORY_MAG_THRES_Y		100.0f
#define LPMS_FACTORY_MAG_THRES_Z		100.0f

#define LPMS_FACTORY_ACC_COMP_GAIN_1		1.0f
#define LPMS_FACTORY_ACC_COVAR_1		0.1f
#define LPMS_FACTORY_MAG_COMP_GAIN_1		1.0f
#define LPMS_FACTORY_MAG_COVAR_1		5.0e2f

#define LPMS_FACTORY_ACC_COMP_GAIN_2		1.0f
#define LPMS_FACTORY_ACC_COVAR_2		0.1f
#define LPMS_FACTORY_MAG_COMP_GAIN_2		1.0f
#define LPMS_FACTORY_MAG_COVAR_2		5.0e1f

#define LPMS_FACTORY_ACC_COMP_GAIN_3		1.0f
#define LPMS_FACTORY_ACC_COVAR_3		0.1f
#define LPMS_FACTORY_MAG_COMP_GAIN_3		1.0f
#define LPMS_FACTORY_MAG_COVAR_3		25.0f

#define LPMS_FACTORY_ACC_COMP_GAIN_4		1.0f
#define LPMS_FACTORY_ACC_COVAR_4		0.1f
#define LPMS_FACTORY_MAG_COMP_GAIN_4		1.0f
#define LPMS_FACTORY_MAG_COVAR_4		0.5f

#define LPMS_FACTORY_ACC_COMP_GAIN_USER		LPMS_FACTORY_ACC_COMP_GAIN_1
#define LPMS_FACTORY_ACC_COVAR_USER		LPMS_FACTORY_ACC_COVAR_1
#define LPMS_FACTORY_MAG_COMP_GAIN_USER		LPMS_FACTORY_MAG_COMP_GAIN_1
#define LPMS_FACTORY_MAG_COVAR_USER		LPMS_FACTORY_MAG_COVAR_1

#define LPMS_FACTORY_PROCESS_COVAR		5.0e-5f

#define LPMS_FACTORY_OFFSET_QUAT_0 		1.0f
#define LPMS_FACTORY_OFFSET_QUAT_1 		0.0f
#define LPMS_FACTORY_OFFSET_QUAT_2 		0.0f
#define LPMS_FACTORY_OFFSET_QUAT_3 		0.0f

#define LPMS_FACTORY_FIELD_EST 			50.0f
#define LPMS_FACTORY_FIELD_INC			30.0f

#define LPMS_FACTORY_GYR_ALIG_BIAS_X		0.0f
#define LPMS_FACTORY_GYR_ALIG_BIAS_Y		0.0f
#define LPMS_FACTORY_GYR_ALIG_BIAS_Z		0.0f

#define LPMS_FACTORY_RAW_DATA_LP 		LPMS_LP_OFF

#define LPMS_FACTORY_CAN_MAPPING_0		0
#define LPMS_FACTORY_CAN_MAPPING_1		1
#define LPMS_FACTORY_CAN_MAPPING_2		2
#define LPMS_FACTORY_CAN_MAPPING_3		3
#define LPMS_FACTORY_CAN_MAPPING_4		4
#define LPMS_FACTORY_CAN_MAPPING_5		5
#define LPMS_FACTORY_CAN_MAPPING_6		6
#define LPMS_FACTORY_CAN_MAPPING_7		7
#define LPMS_FACTORY_CAN_MAPPING_8		8
#define LPMS_FACTORY_CAN_MAPPING_9		9
#define LPMS_FACTORY_CAN_MAPPING_10		10
#define LPMS_FACTORY_CAN_MAPPING_11     	11
#define LPMS_FACTORY_CAN_MAPPING_12		12
#define LPMS_FACTORY_CAN_MAPPING_13		13
#define LPMS_FACTORY_CAN_MAPPING_14		14
#define LPMS_FACTORY_CAN_MAPPING_15		15

#define LPMS_FACTORY_CAN_HEARTBEAT		LPMS_CAN_HEARTBEAT_010

#define LPMS_FACTORY_LIN_ACC_COMP_MODE		2
#define LPMS_FACTORY_CENTRI_COMP_MODE		1

#define LPMS_FACTORY_FILTER_MODE		LPMS_FILTER_GYR_ACC

#define LPMS_FACTORY_CAN_CONFIGURATION		0x05140003

#define LPMS_FACTORY_MAG_ALIG_BIAS_X		0.0f
#define LPMS_FACTORY_MAG_ALIG_BIAS_Y		0.0f
#define LPMS_FACTORY_MAG_ALIG_BIAS_Z		0.0f

#define LPMS_FACTORY_UART_BAUDRATE			(LPMS_UART_BAUDRATE_115200 | LPMS_UART_FORMAT_LPBUS)

#endif