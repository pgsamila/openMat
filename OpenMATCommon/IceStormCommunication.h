/****************************************************************************
**
** Copyright (C) 2011 LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** OpenMAT is free software: you can redistribute it and/or modify it under 
** the terms of the GNU General Public License as published by the Free 
** Software Foundation, either version 3 of the License, or (at your option) 
** any later version.
** 
** OpenMAT is distributed in the hope that it will be useful, but WITHOUT 
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
** FITNESS FOR A PARTICULAR PURPOSE. See the GNU \ General Public License 
** for more details.
** 
** You should have received a copy of the GNU General Public License along 
** with the OpenMAT library. If not, see <http://www.gnu.org/licenses/>.
**
****************************************************************************/

#ifndef ICE_STORM_COMMUNICATION
#define ICE_STORM_COMMUNICATION

#include <QObject>

#ifdef USE_ZEROC_ICE
	#include <Ice/Ice.h>
	#include <IceStorm/IceStorm.h>
#endif	

#include <boost/thread/thread.hpp> 

#include "ImuMonitor.h"
#include "ImuData.h"
#include "LpmsSensorI.h"
#include "MicroMeasure.h"

#include <list>
using namespace std;

using namespace ImuDataCommunication;

class ImuMonitorI : public QObject, virtual public ImuMonitor
{
Q_OBJECT

	public:
		virtual void updateImuData(const OpenMatImuData& id, const Ice::Current&);
		
	signals:
		void newImuData(ImuData ld);
};
	
class IceStormPublisher
{
	public:
		#ifdef USE_ZEROC_ICE
			Ice::CommunicatorPtr communicator;
			Ice::ObjectPrx obj;
			IceStorm::TopicManagerPrx topicManager;
			IceStorm::TopicPrx topic;
			Ice::ObjectPrx pub;
			ImuMonitorPrx imuMonitor;
		#endif
		
		bool isConnected;
		list<LpmsSensorI*> sensorList;
		bool stopped;

	public:
		IceStormPublisher(void);
		~IceStormPublisher(void);		
		void connect(void);
		bool checkConnected(void);
		void disconnect(void);
		void runThread(void);
		void stopThread(void);		
		void addSensor(LpmsSensorI* sensor);
		void removeSensor(LpmsSensorI* sensor);
		void updateImuData(ImuData ld);
};

#endif