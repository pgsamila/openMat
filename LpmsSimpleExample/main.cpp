#include <cstdio>
#include <thread>
#include "LpemgSensorI.h"
#include "LpmsSensorManagerI.h"

int main(int argc, char *argv[])
{
	EmgData ed;

	// Gets a LpmsSensorManager instance
	LpmsSensorManagerI* manager = LpmsSensorManagerFactory();

	// Connects to LPMS-B sensor with address 00:11:22:33:44:55 
	LpSensorBaseI* sensor = manager->addSensor(DEVICE_LPEMG_B, "38:c0:96:41:0b:bb");

	while(1) {		 
		// Checks, if connected
		if (
			sensor->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
			sensor->hasData()
			) {
			
			// Reads quaternion data
			sensor->getSensorData(&ed);

			// Shows data
			printf("Timestamp=%f, U=%f\n", 
				ed.timestamp, ed.u);
		}
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	// Removes the initialized sensor
	manager->removeSensor(sensor);
		
	// Deletes LpmsSensorManager object 
	delete manager;
	
	return 0;
}