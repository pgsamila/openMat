#include "MotionBuilderCommunication.h"
#include "MainWindow.h"


using Eigen::Quaternion;
using Eigen::Matrix3d;
using namespace std;

// Helper functions forward declaration
std::ostream& operator<<(std::ostream& os, const Eigen::Quaternion<double>& q);
std::ostream& operator<<(std::ostream& os, const double arr[3]); 

const string MotionBuilderCommunication::mTag="MBCommunication";

MotionBuilderCommunication::MotionBuilderCommunication(void):
bRunning(false)
{ 
}


MotionBuilderCommunication::~MotionBuilderCommunication(void)
{
	if (bRunning)
		stopServer();
}


void MotionBuilderCommunication::startServer(void){
	std::thread t(&MotionBuilderCommunication::runThread, this);
	t.detach();
	logd("MBServer started");
}

void MotionBuilderCommunication::stopServer(void){
	if (bRunning){
		logd("Stopping MBServer");
		stopThread();
		if (Soc) 
#ifdef _WIN32 
			closesocket( Soc ); 
#else
			close( Soc ); 
#endif
		Cleanup(); 
	}
}

void MotionBuilderCommunication::runThread(void)
{
	Soc=0;
	bRunning = true;
	logd("MBServer Thread Running...");

	while (bRunning){
		static double ServerStartedOn = 0.0;
		if (!Soc){
			Soc = initServer( PORTNUMBER );
			if (Soc) {
				logd("MBServer started on port " + toString(PORTNUMBER));
			}
		} else { 
			logd("Waiting for connection");
			sockaddr_in	lClientAddr;

#ifdef _WIN32
			int	lSize;
#else
			socklen_t lSize;
#endif

			int	lSocket;

			lSize = sizeof(lClientAddr);
			bzero((char *)&lClientAddr, sizeof(lClientAddr));

			lSocket = accept(Soc, (struct sockaddr*)&lClientAddr, &lSize);

#ifdef _WIN32
			if( lSocket != INVALID_SOCKET ) {
#else
			if( lSocket != -1 ) {
#endif

				sockaddr_in	lAddr; 
				 
				bzero((char *)&lAddr, sizeof(lAddr));
				if( getsockname(lSocket, (struct sockaddr*)&lAddr, &lSize) < 0 ) {
					return ;
				}
				logd("Connection established");
				ServerStartedOn = (double)getNanoSeconds();  
				
				string sDeviceInfo = "LPMS " + (string)LPMS_CONTROL_VERSION;
				char deviceInfo[50];
				strcpy( deviceInfo, sDeviceInfo.c_str() ); 

				// Main MB communication bridge
				// send version number to MB
#ifdef _WIN32
				if (send( lSocket, (char*)&deviceInfo, sizeof(deviceInfo), 0)==SOCKET_ERROR){					
#else
				if (send( lSocket, (char*)&deviceInfo, sizeof(deviceInfo), 0)==-1){					
#endif
					logd("Connection error"); 
				}
				else {
					LPMSRotationData rotDat;
					int fps=30;
					int count=0;
					recv(lSocket, (char*)&fps, sizeof(fps), 0); 					
					logd("MBServer FPS: " + toString(fps));
					while (bRunning)
					{  
						updateImuData(rotDat);
						rotDat.mTime = getNanoSeconds(); 

#ifdef _WIN32				 
						if (send( lSocket, (char*)&rotDat,sizeof(rotDat), 0)==SOCKET_ERROR)
#else
						if (send( lSocket, (char*)&rotDat,sizeof(rotDat), 0)==-1)
#endif
							break;
						std::this_thread::sleep_for(std::chrono::microseconds(1000000)/ fps);
						//Sleep( 1000/fps );
					}
				} 
				/*
				else {
					
					list<LpmsSensorI*>::iterator it;	
					int i=0;
					ImuData d;
					ImuData imuVec[2];
					while (bRunning)
					{ 		 
						for (it = sensorList.begin(); it != sensorList.end(); ++it) {
							d = (*it)->getCurrentData(); 
							i=d.openMatId-1;
							if (i < 2){
								imuVec[i]=d;
							}
						}
						if (send( lSocket, (char*)&imuVec, 2*sizeof(ImuData), 0)==SOCKET_ERROR)
							break;  
						
						boost::this_thread::sleep(boost::posix_time::microseconds(1000000)/SIM_FPS);
						//Sleep( 1000);///sSKDataBuffer::SIM_FPS );
					}
				} 
				*/
				shutdown(lSocket, 2);

#ifdef _WIN32
				closesocket(lSocket);
#else
				close(lSocket);
#endif
				
				logd("Connection closed, connection time = "+ toString((getNanoSeconds()-ServerStartedOn)/1000000.0)  +" ms");
			} else {
				logd("Invalid socket");
			}
		}
	} // while

	if (Soc)
#ifdef _WIN32
		closesocket( Soc );
#else
		close( Soc );
#endif
	Cleanup();  
	
	logd("MBServer Thread Terminated =)");
}

void MotionBuilderCommunication::stopThread(void)
{
	bRunning = false;
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void MotionBuilderCommunication::addSensor(LpmsSensorI* sensor)
{ 
	sensorList.push_back(sensor); 
}

void MotionBuilderCommunication::removeSensor(LpmsSensorI* sensor)
{ 
	sensorList.remove(sensor); 
}

// ////////////////////////////////////////////
// MB Server
///////////////////////////////////////////////
bool MotionBuilderCommunication::initialize()
{
#ifdef _WIN32
	WSADATA wsadata;
	if (WSAStartup(WS_VERSION_REQUIRED, &wsadata)) 
	{ 
		Cleanup();
		return false;
	}
#endif
	return true;
}

int MotionBuilderCommunication::initServer(int pPort)
{
	int lSocket;
	struct protoent* lP;
	struct sockaddr_in  lSin;

	initialize();

#ifdef _WIN32
	lP = getprotobyname("tcp");
	lSocket = socket(AF_INET, SOCK_STREAM, lP->p_proto);
#else
	lSocket = socket(AF_INET, SOCK_STREAM, 0);
#endif

	if (lSocket)
	{
		bzero((char *)&lSin, sizeof(lSin));
		lSin.sin_family = AF_INET;
		lSin.sin_port = htons(pPort);
		lSin.sin_addr.s_addr = 0L;

		//Bind socket
		if( ::bind(lSocket, (struct sockaddr*)&lSin, sizeof(lSin)) < 0 )
			return 0;

		if( listen(lSocket, 5) < 0 )
			return 0;
	}
	return lSocket;
}

bool MotionBuilderCommunication::Cleanup()
{
#ifdef _WIN32
	if (WSACleanup()) 
	{ 
		WSACleanup(); 
		return false;
	}
#endif
	return true;
}

void MotionBuilderCommunication::bzero(char *b, size_t length)
{
	memset( b,0,length );
}

#ifndef _WIN32
static const unsigned usec_per_sec = 1000000;
static const unsigned usec_per_msec = 1000;

bool QueryPerformanceFrequency(long long *frequency)
{
    *frequency = usec_per_sec;

    return true;
}	

bool QueryPerformanceCounter(long long *performance_count)
{
    struct timeval time;

    assert(performance_count != NULL);

    gettimeofday(&time, NULL);
    *performance_count = time.tv_usec + time.tv_sec * usec_per_sec;

    return true;
}
#endif

nsTime MotionBuilderCommunication::getNanoSeconds()
{
	static double dmUnits = 0.0;

#ifdef _WIN32
	LARGE_INTEGER	t;
#else
	long long t;
#endif

	if(QueryPerformanceCounter(&t)) 
	{
		double	count;

#ifdef _WIN32
		count = (double) t.QuadPart;
#else
		count = (double) t;
#endif

		if (!dmUnits) 
		{
			QueryPerformanceFrequency(&t);
#ifdef _WIN32
			dmUnits = 1000000000.0 / (double) t.QuadPart;
#else
			dmUnits = 1000000000.0 / (double) t;
#endif
		}
		return (nsTime) (count * dmUnits);
	}
	return 0;
}

////////////////////////////////////////////////////////
// LPMS imu data process
//////////////////////////////////////////////////////// 

// Prepare LPMS sensor data to send to MB.
// Sensor data is inserted into LPMSRotationData.mChannel array
// according to sensor id.
void MotionBuilderCommunication::updateImuData( LPMSRotationData &rd )
{
//	MicroMeasure mm;
	list<LpmsSensorI*>::iterator it;	
	int i=0;
	ImuData d;
	rd.reset();
	rd.deviceOnline=sensorList.size();
	// update measurement 		 
	for (it = sensorList.begin(); it != sensorList.end(); ++it) {
		d = (*it)->getCurrentData();   
		if (i < rd.ChannelCount){
			rd.mChannel[i].id = d.openMatId;
			rd.mChannel[i].q[0] = d.q[0];	// w
			rd.mChannel[i].q[1] = d.q[1];	// x
			rd.mChannel[i].q[2] = d.q[2];	// y
			rd.mChannel[i].q[3] = d.q[3];	// z
			//decodeRotation(rd.mChannel[i].q, d.q[0], d.q[1], d.q[2], d.q[3]);
			i++;
		} else {
			break;
		}
	} 
}

// Decode LPMS quaternion data into XYZ quaternion rotation.
// q*p = p_
void MotionBuilderCommunication::decodeRotation(double dst[], float x, float y, float z, float w){
	Quaternion<double> p_(w,x,y,z);
	Quaternion<double> p(0,1,0,0); 
	Quaternion<double> q; 
	q = p_*p.inverse();
	q.normalize();
	// XYZ original frame 
	dst[0] = q.x();
	dst[1] = q.y();
	dst[2] = q.z();
	dst[3] = q.w();
}


////////////////////////////////////////////////////////
// Helper Functions
//////////////////////////////////////////////////////// 
std::ostream& operator<<(std::ostream& os, const Eigen::Quaternion<double>& q)
{
  os << q.w() << " + " << q.x() << "i + " << q.y() << "j + " << q.z() << "k" ; 
  return os;
}

std::ostream& operator<<(std::ostream& os, const double arr[3])
{
	os << arr[0] << " " << arr[1] << " " << arr[2]; 
  return os;
}

inline void MotionBuilderCommunication::logd(std::string data){
	std::cout << "["+mTag+"] " << data << std::endl;
}

template <typename T>
std::string MotionBuilderCommunication::toString(const T data) const {
	std::stringstream ss;
	ss << data;
	return ss.str();
}
