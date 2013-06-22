/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef CAN_ENGINE
#define CAN_ENGINE

#include <iostream>
#include <string>
#include <queue>
#include <list>

#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>

#ifdef _WIN32
	#include "windows.h"

	#include "PCANBasic.h"
	#include "PCANBasicClass.h"
#endif

#include "LpmsCanIo.h"
#include "DeviceListItem.h"
#include "LpmsDefinitions.h"

class CanEngine {
private:
#ifdef _WIN32
	std::queue<TPCANMsg> txQ;
	TPCANHandle canChannel;
	PCANBasicClass pcan;
#endif

	bool canInitialized;
	std::list<LpmsCanIo *> sensorList;

public:
	/*! Constructor. */
	CanEngine(void);

	/*! Destructor. */
	~CanEngine(void);

	/*! Connect to CAN interface. */
	void connect(void);

	/*! Add one sensor to the list of sensors that are maintained by the CAN engine. */
	void addSensor(LpmsCanIo *s);

	/*! Poll data from CAN bus. */
	void poll(void);

	/*! Close connection to CAN bus interface. */
	void close(void);

	/*! Poll data from transmission queue. */
	bool updateSendQueue(void);

	/*! List CAN devices connected to the system. */
	void listDevices(LpmsDeviceList *v);

	/*! Remove sensor from list of connected sensors. */
	void removeSensor(LpmsCanIo *s);
	
	bool isInterfacePresent(void);
	
	void setBaudrate(int i);
};

#endif
