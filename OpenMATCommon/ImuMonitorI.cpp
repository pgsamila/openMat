#include "ImuMonitorI.h"

void ImuMonitorI::updateImuData(const OpenMatImuData& id, const Ice::Current&) 
{
	ImuData ld;
	
	ld.openMatId = id.openMatId;
	ld.timeStamp = id.timestamp;
	ld.a[0] = id.xAcc;
	ld.a[1] = id.yAcc;
	ld.a[2] = id.zAcc;
	ld.g[0] = id.xGyro;
	ld.g[1] = id.yGyro;
	ld.g[2] = id.zGyro;
	ld.b[0] = id.xMag;
	ld.b[1] = id.yMag;
	ld.b[2] = id.zMag;
	ld.q[0] = id.wQuat;
	ld.q[1] = id.xQuat;
	ld.q[2] = id.yQuat;
	ld.q[3] = id.zQuat;
	ld.r[0] = id.xEuler;
	ld.r[1] = id.yEuler;
	ld.r[2] = id.zEuler;	
	
	for (int i=0; i<9; i++) {
		ld.rotationM[i] = id.rotationM[i];
	}
	
	emit newImuData(ld);
}