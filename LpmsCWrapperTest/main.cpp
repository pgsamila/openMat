#include "stdio.h"

#include "LpSensorCWrapper.h"

int main(int argc, char *argv[])
{
	initializeLpms();
	connectToLpmsCU("A1019SDI");
	// connectToLpmsB("00:06:66:4B:24:F5");
	
	while (1) {
		float o[4];
		getQuaternion(o);
		
		printf("q: %f %f %f %f\n", o[0], o[1], o[2], o[3]);
		printf("e: %f %f %f\n", getEulerX(), getEulerY(), getEulerZ());
		printf("g: %f %f %f\n", getGyrX(), getGyrY(), getGyrZ());
		printf("a: %f %f %f\n", getAccX(), getAccY(), getAccZ());		
		Sleep(500);
	}
	
	return 0;
}