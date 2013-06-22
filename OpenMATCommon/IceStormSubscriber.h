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

#ifndef ICE_STORM_SUBSCRIBER
#define ICE_STORM_SUBSCRIBER

#include <QObject>

#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>

#include <boost/thread/thread.hpp> 

#include "ImuMonitorI.h"
#include "ImuMonitor.h"
#include "ImuData.h"

#include <list>
using namespace std;

using namespace ImuDataCommunication;

class IceStormSubscriber
{
	public:
		Ice::CommunicatorPtr communicator;	
		Ice::ObjectPrx obj;
		IceStorm::TopicManagerPrx topicManager;
		Ice::ObjectAdapterPtr adapter;
		ImuMonitorI *imuMonitor;
		Ice::ObjectPrx proxy;
		IceStorm::TopicPrx topic;
		bool isConnected;		
		
	public:
		IceStormSubscriber(void);
		~IceStormSubscriber(void);
		void connect(void);
		bool checkConnected(void);
		void disconnect(void);	
		void start(void);
};

#endif