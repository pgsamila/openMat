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

#include "MainWindow.h"

QGroupBox *MainWindow::doGraph(void)
{
	QHBoxLayout* graphLayout = new QHBoxLayout();
	
	graphLayout->addWidget(saWindow);
		
	QGroupBox *graphGroupBox = new QGroupBox("Display");
	graphGroupBox->setLayout(graphLayout);	
	
	return graphGroupBox;
}

MainWindow::MainWindow(QWidget *parent)
{	
	std::cout << "[LPMS SanAngeles Demo] Initializing program" << std::endl;	
	
	saWindow = new SanAngelesWindow();
	
	setCentralWidget(saWindow);
	
	this->setMinimumSize(1024, 768);
	showMaximized();
	
	setWindowTitle("LPMS San Angeles Demo");
		
	openSensor();	
		
	QTimer* timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(timerUpdate()));
    timer->start(20);
}

void MainWindow::timerUpdate(void)
{
	ImuData id = lpms->getCurrentData();
	saWindow->updateQuaternion(id);
}

void MainWindow::openSensor(void)
{
	sm = LpmsSensorManagerFactory();
	lpms = sm->addSensor(DEVICE_LPMS_B, "00:06:66:45:dd:0e"); // , "A1019CZF"); // "AE00895J");
}

void MainWindow::closeSensor(void)
{
}

void MainWindow::exitWindow(void)
{
	QMessageBox msgBox;
	int ret;
	
	msgBox.setText("Do you really want to quit?");
	msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox.setDefaultButton(QMessageBox::No);
	ret = msgBox.exec();

	switch (ret) {
		case QMessageBox::Yes:
			closeSensor();
			this->close();
			break;
			
		case QMessageBox::No:
			break;
			
		default:
			break;
	}
}

MainWindow::~MainWindow()
{
}