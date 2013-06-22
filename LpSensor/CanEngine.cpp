/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "CanEngine.h"

CanEngine::CanEngine(void) 
{
	canInitialized = false;
}

CanEngine::~CanEngine(void) 
{
	close();
}

void CanEngine::listDevices(LpmsDeviceList *v)
{	
	if (canInitialized == false) return;

	for (int i=0; i<16; i++) {
		CalibrationData c;
		std::stringstream ss;
		bool f = false;

		ss << i;

		BOOST_FOREACH(LpmsCanIo *cio, sensorList) {
			int p;
			cio->getConfigData()->getParameter(PRM_OPENMAT_ID, &p);
			if (p == i) f = true;
		}
		
		if (f == true) {
			v->push_back(DeviceListItem(ss.str().c_str(), DEVICE_LPMS_C));
			break;
		}

		c.setParameter(PRM_DEVICE_ID, ss.str());

		LpmsCanIo sensor(&c);
		sensor.connect(ss.str());
		addSensor(&sensor);

		boost::this_thread::sleep(boost::posix_time::milliseconds(10));	
		sensor.setCommandMode();
		
		MicroMeasure mm;
		mm.reset();
		
		while (mm.measure() < 500000) {
			poll();
			sensor.checkState();

			if (sensor.isWaitForAck() == false) {
				v->push_back(DeviceListItem(ss.str().c_str(), DEVICE_LPMS_C));
				std::cout << "[LpmsCanIo] Discovered device: " << i << std::endl;
				break;
			}

			// boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}

		sensorList.pop_back();
	}
}

void CanEngine::connect(void) 
{
	TPCANStatus r;

	canChannel = PCAN_USBBUS1;

	if (canInitialized == false) {
		r = pcan.Initialize(canChannel, PCAN_BAUD_1M);
		if (r != PCAN_ERROR_OK) {
			std::cout << "[CanEngine] CAN Interface not detected." << std::endl;
			return;
		}
		
		boost::uint8_t v = PCAN_PARAMETER_ON;
		r = pcan.SetValue(canChannel, PCAN_BUSOFF_AUTORESET, &v, 1);
	}

	canInitialized = true;
}

void CanEngine::addSensor(LpmsCanIo *s)
{
	sensorList.push_back(s);
}

void CanEngine::removeSensor(LpmsCanIo *s)
{
	sensorList.remove(s);
}

void CanEngine::poll() {
	TPCANStatus r;
	TPCANMsg m;
	TPCANTimestamp t;

	if (canInitialized == false) return;

	updateSendQueue();

	r = pcan.Read(canChannel, &m, &t);

	if (r != PCAN_ERROR_OK) {
		return;
	}

	BOOST_FOREACH(LpmsCanIo *c, sensorList) {
		c->parseCanMsg(m);
	}
}

void CanEngine::close() {
	if (canInitialized == true) {
		// CAN_Uninitialize(canChannel);
		pcan.Uninitialize(canChannel);
	}

	canInitialized = false;
}

bool CanEngine::updateSendQueue(void)
{
	TPCANStatus r;		
	TPCANMsg m;

	if (canInitialized == false) return false;

	BOOST_FOREACH(LpmsCanIo *c, sensorList) {
		int rT = 0;
		while (c->getTxMessage(&txQ) == true && rT < 255) ++rT;
	}
	
	long timeout = 0;
	
	if (txQ.size() > 0) {
		m = txQ.front();

		/* printf("Msg. write %x %x %x %x %x %x %x %x\n", m.DATA[0], m.DATA[1], m.DATA[2], m.DATA[3], m.DATA[4], m.DATA[5], m.DATA[6], m.DATA[7]); */
		r = pcan.Write(canChannel, &m);
		
		boost::this_thread::sleep(boost::posix_time::microseconds(2000));
		
		if (r == PCAN_ERROR_QXMTFULL) {
			// std::cout << "[CanEngine] Transmit queue full!" << std::endl;
		}
		
		if (r != PCAN_ERROR_OK && timeout < 0xffff) {
			// std::cout << "[CanEngine] Transmission error!" << std::endl;
			timeout++;
		} else {
			timeout = 0;
			txQ.pop();
		}
	}
	
	return true;
}

bool CanEngine::isInterfacePresent(void)
{
	TPCANStatus r;
	canChannel = PCAN_USBBUS1;

	if (canInitialized == false) {
		r = pcan.Initialize(canChannel, PCAN_BAUD_1M);
		
		if (r != PCAN_ERROR_OK) {
			std::cout << "[CanEngine] CAN Interface not detected." << std::endl;
			
			return false;	
		} 
		
		pcan.Uninitialize(canChannel);
	}
	
	return true;
}

void CanEngine::setBaudrate(int i)
{
	TPCANStatus r;
	canChannel = PCAN_USBBUS1;

	if (canInitialized == true) pcan.Uninitialize(canChannel);
	
	switch (i) {
	case 0:
		r = pcan.Initialize(canChannel, PCAN_BAUD_125K);
	break;
	
	case 1:
		r = pcan.Initialize(canChannel, PCAN_BAUD_250K);
	break;

	case 2:
		r = pcan.Initialize(canChannel, PCAN_BAUD_500K);
	break;

	case 3:	
		r = pcan.Initialize(canChannel, PCAN_BAUD_1M);
	break;
	}
		
	if (r != PCAN_ERROR_OK) {
		std::cout << "[CanEngine] CAN Interface not detected." << std::endl;

		canInitialized = false;
	} 
	
	boost::uint8_t v = PCAN_PARAMETER_ON;
	r = pcan.SetValue(canChannel, PCAN_BUSOFF_AUTORESET, &v, 1);

	canInitialized = true;
}