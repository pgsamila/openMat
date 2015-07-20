#include "stdio.h"

#include "LpSensorCWrapper.h"

int main(int argc, char *argv[])
{
	initializeLpms();
	// connectToLpmsCU("A1019SDI");
	connectToLpmsB("00:04:3E:99:02:6C");
	
	while (1) {
		printf("q: %f %f %f %f\n", getQuaternionW(), getQuaternionX(), getQuaternionY(), getQuaternionZ());
		printf("e: %f %f %f\n", getEulerX(), getEulerY(), getEulerZ());
		printf("g: %f %f %f\n", getGyrX(), getGyrY(), getGyrZ());
		printf("a: %f %f %f\n", getAccX(), getAccY(), getAccZ());		
		Sleep(500);
	}
	
	return 0;
}