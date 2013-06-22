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

#include "IceStormSubscriber.h"

IceStormSubscriber::IceStormSubscriber(void)
{
	isConnected = false;
}

IceStormSubscriber::~IceStormSubscriber(void)
{
	disconnect();
}
	
void IceStormSubscriber::connect(void)
{
	communicator = Ice::initialize();	
	cout << "[IceStorm Subscriber] Initializing IceStorm" << endl;		

	try {	
		obj = communicator->stringToProxy("IceStorm/TopicManager:tcp -p 9999");
		cout << "[IceStorm Subscriber] Connecting to topic manager" << endl;
		topicManager = IceStorm::TopicManagerPrx::checkedCast(obj);		
	} catch (Ice::ConnectionRefusedException&) {
		cout << "[IceStorm Subscriber] Could not connect to topic manager (IceBox not started)" << endl;
		isConnected = false;
		return;
	} /* catch (Ice::ProxyParseException) {
		std::cout << "ProxyParseException" << std::endl;
		return;
	} catch (Ice::EndpointParseException) {
		std::cout << "EndpointParseException" << std::endl;	
		return;
	} catch (Ice::IdentityParseException) {
		std::cout << "IdentityParseException" << std::endl;	
		return;
	} */
	
	try {	
		adapter = communicator->createObjectAdapterWithEndpoints("ImuData.Subscriber", "tcp");	
	} catch (Ice::InitializationException& e) {
		cout << e.what() << endl;
	}

	imuMonitor = new ImuMonitorI;
	proxy = adapter->addWithUUID(imuMonitor)->ice_oneway();
	
	try 
	{
		topic = topicManager->retrieve("ImuData");
	} 
	catch (const IceStorm::NoSuchTopic&) 
	{
		cout << "[IceStorm Subscriber] Topic not found, creating new ImuData topic" << endl;
		topic = topicManager->create("ImuData");
	}
	IceStorm::QoS qos;
	topic->subscribeAndGetPublisher(qos, proxy);

	adapter->activate();
	isConnected = true;
	cout << "[IceStorm Subscriber] IceStorm subscriber started" << endl;	
	communicator->waitForShutdown();
	topic->unsubscribe(proxy);	
}

bool IceStormSubscriber::checkConnected(void)
{
	return isConnected;
}

void IceStormSubscriber::disconnect(void)
{
	if (isConnected == true)
	{
		communicator->shutdown();
		cout << "[IceStorm Subscriber] IceStorm subscriber disconnected" << endl;			
		isConnected = false;	
	}
}

void IceStormSubscriber::start(void)
{
	boost::thread t(&IceStormSubscriber::connect, this);
	t.detach();
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
}
