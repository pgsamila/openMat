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

#include "DiscoveryItem.h"

DiscoveryItem::DiscoveryItem(QTreeWidget* tree, string deviceId, int deviceType, int interfaceType) :
	deviceType(deviceType),
	deviceId(deviceId) {
	QGridLayout* gl = new QGridLayout();

	gl->addWidget(new QLabel("Interface type:"), 1, 0);
	gl->addWidget(new QLabel("Device ID:"), 2, 0);
	
	gl->addWidget(deviceIdItem = new QLabel(deviceId.c_str()), 2, 1);
	
	switch (deviceType) {
	case DEVICE_LPMS_B:
		gl->addWidget(interfaceTypeItem = new QLabel("Bluetooth 2.1"), 1, 1);
		treeItem = new QTreeWidgetItem(tree, QStringList(QString("LPMS-B (") + deviceId.c_str() + ")"));
	break;
	
	case DEVICE_LPMS_BLE:
		gl->addWidget(interfaceTypeItem = new QLabel("Bluetooth LE"), 1, 1);
		treeItem = new QTreeWidgetItem(tree, QStringList(QString("LPMS-BLE (") + deviceId.c_str() + ")"));
	break;

	case DEVICE_LPMS_U:
		gl->addWidget(interfaceTypeItem = new QLabel("USB"), 1, 1);
		treeItem = new QTreeWidgetItem(tree, QStringList(QString("LPMS-CU (USB ID:") + deviceId.c_str() + ")"));
	break;

	case DEVICE_LPMS_C:
		gl->addWidget(interfaceTypeItem = new QLabel("CAN bus"), 1, 1);
		treeItem = new QTreeWidgetItem(tree, QStringList(QString("LPMS-CU (CAN ID: ") + deviceId.c_str() + ")"));
	break;
	
	case DEVICE_LPMS_RS232:
		gl->addWidget(interfaceTypeItem = new QLabel("RS-232"), 1, 1);
		treeItem = new QTreeWidgetItem(tree, QStringList(QString("LPMS-CUR (Port: ") + deviceId.c_str() + ")"));
	break;
	
	case DEVICE_LPEMG_B:
		gl->addWidget(interfaceTypeItem = new QLabel("Bluetooth 2.1"), 1, 1);
		treeItem = new QTreeWidgetItem(tree, QStringList(QString("LPEMG-B (") + deviceId.c_str() + ")"));
	break;	
	}

	QTreeWidgetItem* subTreeItem = new QTreeWidgetItem(treeItem);
	
	setLayout(gl);
	
	tree->setItemWidget(subTreeItem, 0, this);
}

DiscoveryItem::~DiscoveryItem(void)
{
}

int DiscoveryItem::getDeviceType(void) 
{
	return deviceType;
}

string DiscoveryItem::getDeviceId(void) 
{
	return deviceId;
}
	