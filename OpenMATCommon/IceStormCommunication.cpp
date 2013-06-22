/***********************************************************************
** Copyright (C) 2011 LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** Redistribution and use in source and binary forms, with 
** or without modification, are permitted provided that the 
** following conditions are met:
**
** Redistributions of source code must retain the above copyright 
** notice, this list of conditions and the following disclaimer.
** Redistributions in binary form must reproduce the above copyright 
** notice, this list of conditions and the following disclaimer in 
** the documentation and/or other materials provided with the 
** distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
** FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#include "IceStormCommunication.h"

#define OPEN_MAT_UPDATE_RATE 10000	

boost::mutex iceMutex;

IceStormPublisher::IceStormPublisher(void)
{
	isConnected = false;

	communicator = Ice::initialize();	
	
	boost::thread t(&IceStormPublisher::runThread, this);
	t.detach();
}

IceStormPublisher::~IceStormPublisher(void)
{
	stopThread();
	disconnect();
}

void IceStormPublisher::addSensor(LpmsSensorI* sensor)
{
	iceMutex.lock();
	sensorList.push_back(sensor);
	iceMutex.unlock();
}

void IceStormPublisher::removeSensor(LpmsSensorI* sensor)
{
	iceMutex.lock();
	sensorList.remove(sensor);
	iceMutex.unlock();
}
	
void IceStormPublisher::runThread(void)
{
	MicroMeasure mm;
	list<LpmsSensorI*>::iterator i;	
	ImuData d;
	stopped = false;
		
	while (stopped == false) {		
		if (mm.measure() > OPEN_MAT_UPDATE_RATE) {								
			mm.reset();
			
			iceMutex.lock();
			for (i = sensorList.begin(); i != sensorList.end(); i++) {
				d = (*i)->getCurrentData();
				updateImuData(d);
			}
			iceMutex.unlock();
		}
		
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
}

void IceStormPublisher::stopThread(void)
{
	stopped = true;
	boost::this_thread::sleep(boost::posix_time::milliseconds(50));
}

void IceStormPublisher::connect(void)
{
	cout << "[IceStorm Publisher] Initializing IceStorm" << endl;

	try {
		cout << "[IceStorm Publisher] Connecting to topic manager" << endl;	
		obj = communicator->stringToProxy("IceStorm/TopicManager:tcp -p 9999");
		topicManager = IceStorm::TopicManagerPrx::checkedCast(obj);
	} catch (Ice::ConnectionRefusedException&) {
		cout << "[IceStorm Publisher] Could not connect to topic manager (IceBox not started)" << endl;
		isConnected = false;
		return;
	}
	while (!topic) 
	{
		try 
		{
			topic = topicManager->retrieve("ImuData");
		} 
		catch (const IceStorm::NoSuchTopic&) 
		{
			try 
			{
				topic = topicManager->create("ImuData");
			} 
			catch (const IceStorm::TopicExists&) 
			{
				cout << "[IceStorm Publisher] Topic already exists" << endl;
			}
		}
	}	
	pub = topic->getPublisher()->ice_oneway();
	imuMonitor = ImuMonitorPrx::uncheckedCast(pub);
	isConnected = true;
	cout << "[IceStorm Publisher] IceStorm publisher started" << endl;
}

bool IceStormPublisher::checkConnected(void)
{
	return isConnected;
}

void IceStormPublisher::disconnect(void)
{
	stopped = true;
	
	try {
		communicator->shutdown();
		cout << "[IceStorm Publisher] IceStorm publisher disconnected" << endl;		
	} catch (Ice::ObjectAdapterDeactivatedException&) {
	}
	
	isConnected = false;		
}

void IceStormPublisher::updateImuData(ImuData ld)
{
	struct OpenMatImuData id;
	
	if (isConnected == false) return;
	
	id.openMatId = ld.openMatId;
	id.timestamp = ld.timestamp;
	id.xAcc = ld.a[0];
	id.yAcc = ld.a[1];
	id.zAcc = ld.a[2];
	id.xGyro = ld.g[0];
	id.yGyro = ld.g[1];
	id.zGyro = ld.g[2];
	id.xMag = ld.b[0];
	id.yMag = ld.b[1];
	id.zMag = ld.b[2];
	id.wQuat = ld.q[0];
	id.xQuat = ld.q[1];
	id.yQuat = ld.q[2];
	id.zQuat = ld.q[3];
	id.xEuler = ld.r[0];
	id.yEuler = ld.r[1];
	id.zEuler = ld.r[2];
	
	for (int i=0; i<9; i++) {
		id.rotationM.push_back(ld.rotationM[i]);
	}	
	
	imuMonitor->updateImuData(id);
}