#ifndef MOTIONBUILDERCOMMUNICATION_H
#define MOTIONBUILDERCOMMUNICATION_H

#if defined( WIN32 )
typedef unsigned __int64 nsTime;
#else
typedef unsigned long long nsTime;
#endif

#if defined( _USE_MATH_DEFINES )
#else
	#define _USE_MATH_DEFINES // for C++ 
#endif

#include <stdio.h> 
#include <math.h> 
#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include <sstream>

#if defined( WIN32 )
	#define WS_VERSION_REQUIRED 0x0101
	#include <winsock.h>
	#include <windows.h>
	#pragma comment(lib, "ws2_32.lib")
#else
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <sys/time.h>
#endif

// #include <boost/thread/thread.hpp>
#include <Eigen/Geometry> 

//#include "ImuMonitor.h"
#include "ImuData.h"
#include "LpmsSensorI.h"
//#include "MicroMeasure.h" 

struct LPMSRotationData;

class MotionBuilderCommunication
{
public:
	static const std::string mTag;
	/* Constructor */
	MotionBuilderCommunication(void);

	/* Destructor */
	~MotionBuilderCommunication(void);

	/* Start TCP Server to accept MB Connection */
	void startServer(void);

	/* Stop TCP Server */
	void stopServer(void);

	/* Add sensor */
	void addSensor(LpmsSensorI* sensor);

	/* Remove sensor */
	void removeSensor(LpmsSensorI* sensor);
		
private: 
	/* MBServer */
	void runThread(void);
	void stopThread(void);	
	bool initialize();
	int  initServer(int pPort);
	void bzero(char *b, size_t length);
	bool Cleanup();
	nsTime getNanoSeconds();
	
	/* Update IMU data for MB */
	void updateImuData(LPMSRotationData &rd); 

	/* Decode LPMS quaternion information to quaternion rotation */
	void decodeRotation(double dst[], float w, float x, float y, float z);
	
	/* Helper functions */
	void logd(std::string data);
	template <typename T> std::string toString(const T data) const;
	 
private:	
	static const int PORTNUMBER		= 8889;		// Port number for communication
	static const int SIM_FPS		= 30;		// Tested for 30,60,120
	std::list<LpmsSensorI*> sensorList;
	bool bRunning;		// server thread status
	int Soc;
};

struct LPMSRotationData {	
	static const int ChannelCount = 6; // total number of sensors available
	int deviceOnline;	// number of device online
	nsTime 	mTime;
	struct {
		int id;
		double q[4]; // x, y, z, w
	}mChannel[ChannelCount];

	LPMSRotationData(){
		reset();
	}

	void reset(){
		double qIdentity[4] = {0,0,0,1};
		mTime = 0;
		deviceOnline = 0;
		for (int i=0; i<ChannelCount; ++i){
			mChannel[i].id = -1;
			memcpy(mChannel[i].q, qIdentity,sizeof(mChannel[i].q));
		}
	}
};
#endif
