#include "RawImuMonitorI.h"

const float r2d = 57.2958f;	

void RawImuMonitorI::updateImuData(const RawImuData& id, const Ice::Current&) 
{
	currentData.a[0] = id.xAcc;
	currentData.a[1] = id.yAcc;
	currentData.a[2] = id.zAcc;
	currentData.g[0] = id.xGyro * r2d;
	currentData.g[1] = id.yGyro * r2d;
	currentData.g[2] = id.zGyro * r2d;
	currentData.b[0] = id.xMag;
	currentData.b[1] = id.yMag;
	currentData.b[2] = id.zMag;
}

ImuData RawImuMonitorI::getData(void) 
{
	return currentData;
}