#include "stdio.h"

#include <iostream>
#include <sstream>
#include <thread>

#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
#include "MicroMeasure.h"

using namespace std;

// Forward Declarations
void printMenu(void);

// Send data thread
bool threadRunning = false;
bool stopThread = false;

MicroMeasure mm;

void streamDataTask(float t)
{
	ImuData d;

	int count = 0;
	bool runOnce = true; 

	float TotalRunTime = t;
	float epsilon = 0.001f;
	float sensorTimeStampOffset	= 0;
	float sensor2TimeStampOffset = 0;	
	float lastTimeStampSensor = 0;
	long long startTimeStampMM = 0;
	long long lastTimeStampMM = 0;
	long long timeStampMM = 0;

	// Gets a LpmsSensorManager instance
	LpmsSensorManagerI* manager = LpmsSensorManagerFactory();
	LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_B, "00:06:66:4B:25:03");

	threadRunning = true;
	
	mm.reset();
	while (!stopThread) { 

		// Checks, if conncted
		if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED) {
			if (runOnce) {

				// 1s buffer 
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}
			
			d = lpms->getCurrentData();			
			
			if (runOnce) {				
				startTimeStampMM = timeGetTime();
				sensorTimeStampOffset = d.timeStamp;
				
				runOnce = false;
			}
						
			if ((timeGetTime() - startTimeStampMM) / 1000.0f > TotalRunTime)
				stopThread = true;

			if (d.timeStamp - lastTimeStampSensor > epsilon) {
				timeStampMM = timeGetTime();
				
				printf("i= %d, T=%f, Timestamp=%f, dt1(s)=%f, dtSensor(s)=%f\n", 
					count, 
					(timeStampMM - startTimeStampMM) / 1000.0f, 
					d.timeStamp-sensorTimeStampOffset, 
					((timeStampMM - startTimeStampMM) / 1000.0f) - (d.timeStamp-sensorTimeStampOffset),
					d.timeStamp - lastTimeStampSensor
				);
				
				lastTimeStampSensor = d.timeStamp;
				lastTimeStampMM  = timeStampMM;
				
				count++;
			}			
		} else {			
			mm.reset();
		} 
	}
	
	cout << "Total Elapsed time: " <<  (timeGetTime() - startTimeStampMM) / 1000.0f << endl;

	// Removes the initialized sensor
	manager->removeSensor(lpms);
		
	// Deletes LpmsSensorManager object 
	delete manager;

	threadRunning = false;
    cout << "[streamDataThread] Thread Terminated" << endl;
}

int main(int argc, char *argv[])
{					
	string	input;
	bool quit(false); 

	printMenu();
	while(!quit) {		
		cout << "[Main] Enter command: ";
		
		getline(cin, input);
		switch (input[0]) {

		// connect to server
		case 's':
			if (!threadRunning) {
				float runTime;
				
				cout << "Enter total run time(s): ";		

				getline(cin, input);
				stringstream ss(input);
				
				ss >> runTime; 
				stopThread = false;

				std::thread t(streamDataTask, runTime);
				t.detach();
			} else {
				cout << "[Main] Another thread is running\n";
			}
		break;
			
		// disconnect from server
		case 'x':
			stopThread = true;
		break;

		// quit program
		case 'q':
			cout << "[Main] Quiting main program" << endl;
			quit = true;
			stopThread = true;
			break;

			// default
		default:
			cout << "[Main] Unknown command. Try again" << endl;
		break;
		}
	}
	
	// wait for thread to terminate
	cout << "[Main] Terminating...\n";
	while (threadRunning) std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	cout << "[Main] bye\n";
	return 0;
}

	
void printMenu(void)
{
	cout << "*Main menu\n"  
		 << "----------\n" 
		 << "[s] Start data streaming\n"
		 << "[x] Stop data streaming\n"
		 << "[q] Quit\n";		
}