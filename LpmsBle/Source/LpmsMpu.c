#include "LpmsMpu.h"

#define PRINT_ACCEL 0x01
#define PRINT_GYRO 0x02
#define PRINT_QUAT 0x04

#define ACCEL_ON 0x01
#define GYRO_ON 0x02

#define MOTION 0
#define NO_MOTION 1
#define DEFAULT_MPU_HZ 100

unsigned char accel_fsr;
unsigned short gyro_rate, gyro_fsr;
unsigned char sensors_uc;
unsigned char more;
unsigned long sensor_timestamp;
unsigned long mpu_timestamp;
short mpuGyro[3], mpuAccel[3];
long mpuQuat[4];
LpVector4f mpuQuatFloat;

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s {
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	volatile unsigned char new_gyro;
	unsigned short report;
	unsigned short dmp_features;
	unsigned char motion_int_mode;
	struct rx_s rx;
};

static struct hal_s hal = { 0 };

extern LpVector4f q;
extern LpVector3f aRaw;
extern LpVector3f a;
extern LpVector3f gRaw;
extern LpVector4f qAfterOffset;
extern LpVector3f rAfterOffset;
extern LpmsReg gReg;
extern uint32_t measurementTime;
extern uint32_t pressureTime;
extern float canHeartbeatTime;
extern uint32_t measurementTime;
extern float heaveTime;
extern uint16_t dmpFifoRate;
extern LpVector3f gyrRawData;
extern LpVector3f accRawData;
extern LpVector3f magRawData;

void initMpu(void) 
{
	memset(&hal, 0, sizeof(hal));	
	
	mpu_i2c_init();
	mpu_init();
	
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL /*| INV_XYZ_COMPASS */);

	mpu_set_sample_rate(100);

	msDelay(100);

	mpu_set_gyro_fsr(2000);
	mpu_set_accel_fsr(4);
	mpu_set_lpf(42);

	mpu_set_compass_sample_rate(10);

	mpu_set_dmp_state(0);

	msDelay(100);
	
	// mpu_reg_dump();
	
	long gyro;
	long accel;
	
	mpu_run_self_test(&gyro, &accel);	
	
#ifdef USE_DMP	
	hal.sensors = ACCEL_ON | GYRO_ON;
	hal.report = PRINT_QUAT;

	dmp_load_motion_driver_firmware();

	msDelay(10);

	dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));

	msDelay(10);

	hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
	
	dmp_enable_feature(hal.dmp_features);

	msDelay(10);

	dmp_set_fifo_rate(dmpFifoRate);
	
	msDelay(10);

	mpu_set_dmp_state(1);

	msDelay(10);

	hal.dmp_on = 1;
#endif
}

void pollDmp(void)
{
	short gyro[3], accel[3], mag[3];
	long quat[4];
	int i;

	if (hal.dmp_on) {			 
		short sensors;

		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

		if (!more) hal.new_gyro = 0;

		if (sensors & INV_XYZ_GYRO) {
			for (i=0; i<3; ++i) gyrRawData.data[i] = (float) gyro[i];
		}
		
		if (sensors & INV_XYZ_ACCEL) {			
		  	for (i=0; i<3; ++i) accRawData.data[i] = (float) accel[i];
		}

		if (sensors & INV_XYZ_COMPASS) {
		  	for (i=0; i<3; ++i) magRawData.data[i] = (float) accel[i];
		}
		
		if (sensors & INV_WXYZ_QUAT) {	
			for (i=0; i<4; ++i) mpuQuat[i] = quat[i];
		}
	} else {
		if (mpu_get_gyro_reg(gyro, &sensor_timestamp) > -1) {
			for (i=0; i<3; ++i) gyrRawData.data[i] = (float) gyro[i];
		}
		
		if (mpu_get_accel_reg(accel, &sensor_timestamp) > -1) {
		  	for (i=0; i<3; ++i) accRawData.data[i] = (float) accel[i];			
		}

		/* if (mpu_get_compass_reg(mag, &sensor_timestamp) > -1) {
		  	for (i=0; i<3; ++i) magRawData.data[i] = (float) mag[i];
		} */
	}
}

void runSelfTest(void)
{
	int result;
	long gyro[3], accel[3];
	unsigned short accel_sens;

	result = mpu_run_self_test(gyro, accel);

	if ((result & 0x2) == 0x2) {		
		mpu_get_accel_sens(&accel_sens);
		
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		
		mpu_set_accel_bias_6050_reg(accel);
	}
}