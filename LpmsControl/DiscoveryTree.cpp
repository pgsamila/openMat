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

#include "DiscoveryTree.h"
 	
DiscoveryTree::DiscoveryTree(LpmsSensorManagerI* sm, string title, QWidget *mainWin) : 
	QTreeWidget(),
	mainWin(mainWin),
	sm(sm) {
	setColumnCount(1);
	setHeaderLabel(QString(title.c_str()));
	
	selectionModel()->setCurrentIndex(QModelIndex(), QItemSelectionModel::Clear);
	setVerticalScrollMode(QTreeView::ScrollPerPixel);

	deviceVector.clear();

	connect(this, SIGNAL(devicesChanged()), this, SLOT(updateDevices()));
	
	isDiscovering = false;
	scan_serial_ports_ = false;
}

void DiscoveryTree::startDiscoverDevices(bool scan_serial_ports) {
	if (isDiscovering == true) return;
	isDiscovering = true;
	scan_serial_ports_ = scan_serial_ports;

	deviceVector.clear();

	discoverProgress = new QProgressDialog("Discovering LPMS devices...", QString() /*"Cancel"*/, 0, 40, mainWin);
	discoverProgress->setWindowFlags(discoverProgress->windowFlags() & ~Qt::WindowContextHelpButtonHint);
    discoverProgress->setWindowModality(Qt::WindowModal);
	discoverProgress->setMinimumWidth(400);
	discoverProgress->setAutoReset(false);
	discoverProgress->show();

	std::thread t(&DiscoveryTree::discoverDevices, this);
	t.detach();

	progressTimer = new QTimer(this);
	connect(progressTimer, SIGNAL(timeout()), this, SLOT(progressTimerUpdate()));
    progressTimer->start(1000);

	progressCount = 0;
}

void DiscoveryTree::progressTimerUpdate(void)
{
	++progressCount;
	discoverProgress->setValue(progressCount);
	discoverProgress->show();
}

void DiscoveryTree::updateDevices(void) {
	clear();

	for (int i=0; i<deviceVector.nDevices; i++) {
		DiscoveryItem *c = new DiscoveryItem(this, 	deviceVector.device[i].deviceId, deviceVector.device[i].deviceType, 0);

		addTopLevelItem(c->treeItem);	
		setCurrentItem(c->treeItem);
		
		c->treeItem->setExpanded(true);
	}	

	delete progressTimer;
    discoverProgress->close();

	isDiscovering = false;
	update();
}

void DiscoveryTree::discoverDevices(void) 
{
	sm->startListDevices(scan_serial_ports_);

	while (sm->listDevicesBusy() == true) {
        if (discoverProgress->wasCanceled()) {
			sm->stopListDevices();
			break;
		}
	}
	
	deviceVector = sm->getDeviceList();

	emit devicesChanged();
}

void DiscoveryTree::writeToFile(string fn)
{
	ofstream f(fn.c_str());

	if (f.is_open()) {
		for (int i=0; i<topLevelItemCount(); i++) {
			try {
				f << ((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceType() << " " << ((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceId() << std::endl;
			} catch (ofstream::failure e) {
				break;
			}
		}	
		f.close();
	}
}

void DiscoveryTree::readFromFile(string fn)
{
	int deviceType;
	std::string deviceId;
	
	ifstream f(fn.c_str());
	
	if (f.is_open()) {
		while (f.eof() == false) {
			try {
				f >> deviceType >> ws >> deviceId;
			} catch (ifstream::failure e) {
				break;
			}
			if (deviceType == DEVICE_LPMS_B || deviceType == DEVICE_LPMS_BLE || deviceType == DEVICE_LPMS_U || deviceType == DEVICE_LPMS_C || deviceType == DEVICE_LPMS_RS232 || deviceType == DEVICE_LPMS_TCP) {
				addDevice(deviceType, deviceId);
			}
		}

		f.close();
	}
}

void DiscoveryTree::addDevice(int deviceType, string deviceId)
{
	for (int i=0; i < topLevelItemCount(); i++) if (((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceId() == deviceId) return;

	DiscoveryItem *c = new DiscoveryItem(this, deviceId, deviceType, 0);

	addTopLevelItem(c->treeItem);	
	setCurrentItem(c->treeItem);
	c->treeItem->setExpanded(true);	
}

void DiscoveryTree::removeCurrentDevice(void)
{
	if (topLevelItemCount() > 0) {
		// std::cout << indexOfTopLevelItem(currentItem()) << std::endl;
		if (indexOfTopLevelItem(currentItem()) == -1) {
			takeTopLevelItem(indexOfTopLevelItem(currentItem()->parent()));
		} else {
			takeTopLevelItem(indexOfTopLevelItem(currentItem()));
		}
	}
}

int DiscoveryTree::getCurrentDeviceType(void) 
{
	DiscoveryItem *di;

	QTreeWidgetItem *wi = currentItem();
	if (wi->childCount() > 0) {
		QTreeWidgetItem *si = wi->child(0);
		di = (DiscoveryItem*)itemWidget(si, 0);
	} else {
		di = (DiscoveryItem*)itemWidget(wi, 0);
	}
	
	return di->getDeviceType();
}

string DiscoveryTree::getCurrentDeviceId(void)
{
	DiscoveryItem *di;

	QTreeWidgetItem *wi = currentItem();
	if (wi->childCount() > 0) {
		QTreeWidgetItem *si = wi->child(0);
		di = (DiscoveryItem*)itemWidget(si, 0);
	} else {
		di = (DiscoveryItem*)itemWidget(wi, 0);
	}

	return di->getDeviceId();
}

void DiscoveryTree::copy(DiscoveryTree* dt)
{
	clear();

	for (int i=0; i < dt->topLevelItemCount(); i++) {
		addDevice(((DiscoveryItem*)dt->itemWidget(dt->topLevelItem(i)->child(0), 0))->getDeviceType(), ((DiscoveryItem*)dt->itemWidget(dt->topLevelItem(i)->child(0), 0))->getDeviceId());
		std::cout << "copy" << std::endl; 
	}
	
	dt->update();
}

void DiscoveryTree::copyTo(QComboBox *cb)
{
	cb->clear();
	
	for (int i=0; i < topLevelItemCount(); i++) {
		if (((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceType() == DEVICE_LPMS_U) {
			cb->addItem(QString("LPMS-CU (USB ID: ") + QString(((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceId().c_str()) + QString(")"));
		} else if (((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceType() == DEVICE_LPMS_C) {
			cb->addItem(QString("LPMS-CU (CAN ID: ") + QString(((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceId().c_str()) + QString(")"));
		} else if (((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceType() == DEVICE_LPMS_BLE) {
			cb->addItem(QString("LPMS-BLE (") + QString(((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceId().c_str()) + QString(")"));
		} else if (((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceType() == DEVICE_LPMS_B) {
			cb->addItem(QString("LPMS-B (") + QString(((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceId().c_str()) + QString(")"));
		} else if (((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceType() == DEVICE_LPMS_TCP) {
			cb->addItem(QString("LPMS-TCP (") + QString(((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceId().c_str()) + QString(")"));			
		} else {
			cb->addItem(QString("LPMS-RS232 (") + QString(((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceId().c_str()) + QString(")"));
		}
	}
	
	cb->update();
}

void DiscoveryTree::copyTo(LpmsDeviceList *ldl)
{
	ldl->nDevices = 0;
	
	for (int i=0; i < topLevelItemCount(); i++) {
		++ldl->nDevices;
		strcpy(ldl->device[i].deviceId, ((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceId().c_str());
		ldl->device[i].deviceType = ((DiscoveryItem*)itemWidget(topLevelItem(i)->child(0), 0))->getDeviceType();
	}
}