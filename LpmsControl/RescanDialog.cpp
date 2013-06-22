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

#include "RescanDialog.h"

RescanDialog::RescanDialog(LpmsSensorManagerI *sm, QComboBox *comboDeviceList, LpmsDeviceList *deviceList, QWidget* parent) :
	QDialog(parent),
	comboDeviceList(comboDeviceList),
	deviceList(deviceList) {
	this->sm = sm;
	
	QVBoxLayout *v0 = new QVBoxLayout();
	QHBoxLayout *h0 = new QHBoxLayout();
	QHBoxLayout *h1 = new QHBoxLayout();
	
	preferredDevices = new DiscoveryTree(sm, "Preferred devices", this);
	detectedDevices = new DiscoveryTree(sm, "Discovered devices", this);
	
	v0->addWidget(detectedDevices);
	v0->addWidget(preferredDevices);
	
	QPushButton *addDeviceB = new QPushButton("Add device");
	QPushButton *removeDeviceB = new QPushButton("Remove device");
	QPushButton *saveDevicesB = new QPushButton("Save devices");
	QPushButton *rescanDevicesB = new QPushButton("Scan devices");
	
	h0->addWidget(addDeviceB);
	h0->addWidget(removeDeviceB);
	
	h1->addWidget(saveDevicesB);
	h1->addWidget(rescanDevicesB);
	
	v0->addLayout(h0);
	v0->addLayout(h1);
		
	this->setLayout(v0);
	
	// detectedDevices->startDiscoverDevices();
	
	connect(addDeviceB, SIGNAL(clicked()), this, SLOT(addPreferredDevice()));
	connect(removeDeviceB, SIGNAL(clicked()), this, SLOT(removePreferredDevice()));
	connect(saveDevicesB, SIGNAL(clicked()), this, SLOT(saveDevices()));
	connect(rescanDevicesB, SIGNAL(clicked()), this, SLOT(startRescan()));
	
	preferredDevices->readFromFile(PREFERRED_DEVICES_FILE);
	
	preferredDevices->copyTo(comboDeviceList);
	preferredDevices->copyTo(deviceList);
	
	if (deviceList->nDevices == 0) {
		comboDeviceList->addItem(QString("Please add devices.."));
	}
	
	setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
}

void RescanDialog::saveDevices(void)
{
	if (preferredDevices->topLevelItemCount() == 0) return;

	preferredDevices->writeToFile(PREFERRED_DEVICES_FILE);
	preferredDevices->copyTo(comboDeviceList);
	preferredDevices->copyTo(deviceList);

	close();
}

void RescanDialog::startRescan(void)
{
	detectedDevices->startDiscoverDevices();
}

void RescanDialog::addPreferredDevice(void)
{
	if (detectedDevices->topLevelItemCount() == 0) return;

	preferredDevices->addDevice(detectedDevices->getCurrentDeviceType(), detectedDevices->getCurrentDeviceId());
}

void RescanDialog::removePreferredDevice(void)
{
	preferredDevices->removeCurrentDevice();
}
	
void RescanDialog::saveToFile(string fn)
{
}