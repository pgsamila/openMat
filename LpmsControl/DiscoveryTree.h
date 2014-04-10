/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
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

#ifndef DISCOVERY_TREE
#define DISCOVERY_TREE

#include <QTreeWidgetItem>
#include <QTreeWidget>
#include <QProgressDialog>
#include <QTimer>

#ifdef _WIN32
	#include "windows.h"
#endif

#include "DiscoveryItem.h"
#include "LpmsSensorManagerI.h"
#include "DeviceListItem.h"

#include <thread>
#include <list>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#define PREFERRED_DEVICES_FILE "LpmsControlPreferredDevices.txt"

class DiscoveryTree : public QTreeWidget {
Q_OBJECT

public:
	DiscoveryTree(LpmsSensorManagerI *sm, string title, QWidget *mainWin);
	int getCurrentDeviceType(void);
	string getCurrentDeviceId(void);
	void addDevice(int deviceType, string deviceId);
	// void addDevice(DeviceListItem* i);
	void writeToFile(string fn);
	void readFromFile(string fn);
	void removeCurrentDevice(void);
	void copy(DiscoveryTree* dt);
	void copyTo(QComboBox *cb);
	void copyTo(LpmsDeviceList *ldl);

	LpmsDeviceList deviceVector;
	LpmsSensorManagerI* sm;
	QProgressDialog* discoverProgress;

	QTimer* progressTimer;
	int progressCount;
	QWidget* mainWin;
	bool isDiscovering;
	bool scan_serial_ports_;
	
public slots:
	void discoverDevices(void);	
	void startDiscoverDevices(bool scan_serial_ports);
	void updateDevices(void);
	void progressTimerUpdate(void);

signals:
	void devicesChanged(void);
};

#endif