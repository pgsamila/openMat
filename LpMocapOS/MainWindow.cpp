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
	
MainWindow::MainWindow(QWidget *parent) :
	selectedLink(0),
	recordButton(new QPushButton("Record motion data")),
	loadButton(new QPushButton("Load motion data")),
	playButton(new QPushButton("Play")),
	pauseButton(new QPushButton("Stop")),
	resetButton(new QPushButton("Reset")),
	exitButton(new QPushButton("Exit")),
	connectButton(new QPushButton("Connect OpenMAT network")),
	iss(new IceStormSubscriber()),
	hm(new HumanModel()),
	mp(new MotionPlayer(hm)),
	hmWin(new HumanModelWindow(hm)),
	graphWin(new GraphWindow()),
	humanModelTree(new QTreeWidget())
{	
	int iLink = 0;
	int iJoint = 0;	
	int i = 0;	
	
	QPushButton *csvButton = new QPushButton("Export to CSV file");	
	QPushButton *offsetButton = new QPushButton("Reset sensor offset");		
	
 	humanModelTree->setColumnCount(1);
	humanModelTree->setHeaderLabel(QString("Human model"));	
	QTreeWidgetItem *modelRoot = new QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Model root")));	
	humanModelTree->insertTopLevelItem(0, modelRoot);	
	BOOST_FOREACH(Link* l, hm->linkList) {	
		ModelTreeLink *newLink = new ModelTreeLink(l, iLink);
		BOOST_FOREACH(Joint *j, l->jointList) {	
			ModelTreeJoint *newJoint = new ModelTreeJoint(j, iJoint);
			newLink->addChild(newJoint);
			newJoint->setExpanded(true);
			treeJointList.push_back(newJoint);
			++iJoint;
		}
		++iLink;
		modelRoot->addChild(newLink);		
		newLink->setExpanded(true);
		treeLinkList.push_back(newLink);		
	}
	modelRoot->setExpanded(true);
	humanModelTree->setCurrentItem((QTreeWidgetItem *)treeLinkList[0]);
	selectedLink = treeLinkList[0]->itemLink;

	QHBoxLayout* h1 = new QHBoxLayout();
	h1->addWidget(playButton);
	h1->addWidget(resetButton);

	QGridLayout *statusLayout = new QGridLayout();
	statusLayout->addWidget(new QLabel("Recording file:"), i, 0);
	recordingFileLabel = new QLabel("n/a");
	statusLayout->addWidget(recordingFileLabel, i, 1);
	++i;	
	statusLayout->addWidget(new QLabel("Recording time:"), i, 0);
	recordingTimeLabel = new QLabel("<font color='green'>0h 0m 0s 0ms</font>");
	statusLayout->addWidget(recordingTimeLabel, i, 1);
	++i;	
	statusLayout->addWidget(new QLabel("Playback file:"), i, 0);
	playbackFileLabel = new QLabel("n/a");
	statusLayout->addWidget(playbackFileLabel, i, 1);
	++i;	
	statusLayout->addWidget(new QLabel("Playback time:"), i, 0);
	playbackTimeLabel = new QLabel("<font color='blue'>0h 0m 0s 0ms</font>");
	statusLayout->addWidget(playbackTimeLabel, i, 1);
	++i;

	QVBoxLayout *v0 = new QVBoxLayout();
	v0->addWidget(new QLabel("<u>OpenMAT network</u>"));
	v0->addWidget(connectButton);
	v0->addWidget(new QLabel("<u>Human model configuration</u>"));
	v0->addWidget(humanModelTree);
	v0->addWidget(offsetButton);
	v0->addWidget(new QLabel("<u>Motion recording</u>"));
	v0->addLayout(statusLayout);
	v0->addWidget(recordButton);
	v0->addWidget(loadButton);
	v0->addLayout(h1);
	v0->addWidget(csvButton);		
	v0->addWidget(new QLabel("<u>Window control</u>"));
	v0->addWidget(exitButton);
	QGroupBox *g0 = new QGroupBox("Control");
	g0->setLayout(v0);	
	
	QVBoxLayout *v1 = new QVBoxLayout();
	v1->addWidget(hmWin);
	QGroupBox *g1 = new QGroupBox("3D Window");
	g1->setLayout(v1);	

	QVBoxLayout *v2 = new QVBoxLayout();
	v2->addWidget(graphWin);
	QGroupBox *g2 = new QGroupBox("Graph");
	g2->setLayout(v2);	
		
	/*QSplitter *s0 = new QSplitter(Qt::Vertical);
	s0->addWidget(g1);
	s0->addWidget(g2);*/
	
	QSplitter *s1 = new QSplitter();
	s1->addWidget(g0);
	s1->addWidget(g1);
	s1->addWidget(g2);
	s1->setStretchFactor(0, 8);
	s1->setStretchFactor(1, 10);
	s1->setStretchFactor(2, 15);
	
	QHBoxLayout *h0 = new QHBoxLayout();
	h0->addWidget(s1);
	setLayout(h0);
	
	QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateWindow()));
    timer->start(40);
	
	connect(connectButton, SIGNAL(clicked()), this, SLOT(connectServer()));	
	connect(recordButton, SIGNAL(clicked()), this, SLOT(recordMotion()));
	connect(loadButton, SIGNAL(clicked()), this, SLOT(loadMotion()));
	connect(playButton, SIGNAL(clicked()), this, SLOT(playMotion()));
	connect(resetButton, SIGNAL(clicked()), mp, SLOT(reset()));	
	connect(exitButton, SIGNAL(clicked()), this, SLOT(exitWindow()));
	connect(csvButton, SIGNAL(clicked()), this, SLOT(exportCsv()));	
	connect(humanModelTree, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem *)),
		this, SLOT(treeItemChanged(QTreeWidgetItem*, QTreeWidgetItem *)));
	connect(offsetButton, SIGNAL(clicked()), this, SLOT(resetOffset()));
	
	connectServer();
}

void MainWindow::resetOffset(void)
{
	hm->resetOffset();
}

void MainWindow::recordMotion(void)
{
	if (mp->isRecording() == false) {
		QString qfilename = QFileDialog::getSaveFileName(this, "Save motion data", "./", "OpenMAT motion data file (*.omd)");
		string record_filename = qfilename.toStdString();
		if (!(record_filename == "")) {
			if (mp->recordMotionDataFile(record_filename) == true) {
				recordButton->setStyleSheet("QPushButton { color: red; }");
				recordButton->setText("Stop recording");
			}
		}
	} else {
		mp->stopRecording();
		recordButton->setStyleSheet("QPushButton { color: black; }");
		recordButton->setText("Record motion");		
	}
}

void MainWindow::loadMotion(void)
{
	QString qfilename = QFileDialog::getOpenFileName(this, "Load motion data", "./", "OpenMAT motion data file (*.omd)");
	string play_filename = qfilename.toStdString();
	
	if (!(play_filename == "")) {
		if (mp->readMotionDataFile(play_filename) == true) {
			playButton->setStyleSheet("QPushButton { color: red; }");
			playButton->setText("Stop");		
		}
	}
}

void MainWindow::playMotion(void)
{
	if (mp->isPlaying() == false) {
		mp->play();
	} else {
		mp->pause();
	}
}

QString MainWindow::getPlaybackTimestamp(void)
{
	QString ts;
	
	double t = mp->currentPlayTime();
	long h = (long) t / 1000.0 / 60.0 / 60.0;
	long m = (long)		(t 	- ((double) h * 60.0 * 60.0 * 1000.0)) / 1000.0 / 60.0;
	long s = 			(t 	- ((double) h * 60.0 * 60.0 * 1000.0) 
							- ((double) m * 60.0 * 1000.0)) / 1000;
	long ms = (long)	(t 	- ((double) h * 60.0 * 60.0 * 1000.0) 
							- ((double) m * 60.0 * 1000.0) 
							- ((double) s * 1000.0));
			
	ts = QString("<font color='blue'>%1h %2m %3s %4ms</font>").arg(h).arg(m).arg(s).arg(ms);
	return ts;
}

QString MainWindow::getRecordingTimestamp(void)
{
	QString ts;
	
	double t = mp->currentRecordTime();
	long h = (long) t / 1000.0 / 60.0 / 60.0;
	long m = (long)		(t 	- ((double) h * 60.0 * 60.0 * 1000.0)) / 1000.0 / 60.0;
	long s = 			(t 	- ((double) h * 60.0 * 60.0 * 1000.0) 
							- ((double) m * 60.0 * 1000.0)) / 1000;
	long ms = (long)	(t 	- ((double) h * 60.0 * 60.0 * 1000.0) 
							- ((double) m * 60.0 * 1000.0) 
							- ((double) s * 1000.0));
			
	ts = QString("<font color='green'>%1h %2m %3s %4ms</font>").arg(h).arg(m).arg(s).arg(ms);
	return ts;
}

void MainWindow::updateWindow(void)
{
	BOOST_FOREACH(ModelTreeJoint* j, treeJointList) {
		j->update();
		j->setExpanded(true);
	}
	
	BOOST_FOREACH(ModelTreeLink* l, treeLinkList) {	
		l->update();
		l->setExpanded(true);
	}	
	
	playbackTimeLabel->setText(getPlaybackTimestamp());
	playbackFileLabel->setText(mp->getPlaybackFile().c_str());
	
	if (mp->isRecording() == true) {
		recordingTimeLabel->setText(getRecordingTimestamp());
		playbackFileLabel->setText(mp->getRecordingFile().c_str());	
	}
	
	if (mp->isPlaying() == true) {
		playButton->setStyleSheet("QPushButton { color: red; }");
		playButton->setText("Stop");		
	} else {
		playButton->setStyleSheet("QPushButton { color: black; }");
		playButton->setText("Play");		
	}	
	
	if (selectedLink != 0) {
		graphWin->plotData(selectedLink->planeAngle);
	}
}

void MainWindow::exportCsv(void)
{
	if (mp->isRecording() == false) {
		QString qfilename = QFileDialog::getSaveFileName(this, "Export CSV data", "./", "CSV file (*.csv)");
		string csv_filename = qfilename.toStdString();
		if (!(csv_filename == "")) {
			if (mp->writeCSVData(csv_filename) == true) {
				cout << "[Motion data recorder] CSV data has been successfully saved." << endl;
			} else {
				cout << "[Motion data recorder] Saving CSV file has failed." << endl;
			}
		}
	}
}
	
MainWindow::~MainWindow()
{
}

void MainWindow::connectServer(void)
{
	if (iss->checkConnected() == true) {
		iss->disconnect();
		connectButton->setStyleSheet("QPushButton { color: black; }");	
		connectButton->setText("Connect OpenMAT network	");
	} else {
		iss->start();
		if (iss->checkConnected() == true)
		{
			QObject::connect(iss->imuMonitor, SIGNAL(newImuData(ImuData)), this, SLOT(updateImuData(ImuData)));	
			connectButton->setStyleSheet("QPushButton { color: red; }");	
			connectButton->setText("Disconnect OpenMAT network");			
		}
	}
}

void MainWindow::treeItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous)
{
	QTreeWidgetItem* i;
	i = humanModelTree->currentItem();
	
	BOOST_FOREACH(ModelTreeLink* l, treeLinkList) {
		if (i == (QTreeWidgetItem*) l) {
			selectedLink = l->itemLink;
		}
	}
}

void MainWindow::updateImuData(ImuData ld)
{
	hm->updateSensorData(ld);
	mp->newMotionData();
}

void MainWindow::exitWindow(void)
{
	QMessageBox msgBox;
	int ret;
	
	msgBox.setText("Do you really want to quit?");
	msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox.setDefaultButton(QMessageBox::No);
	ret = msgBox.exec();

	switch (ret) 
	{
		case QMessageBox::Yes:
			this->close();
			break;
			
		case QMessageBox::No:
			break;
			
		default:
			break;
	}
}

ModelTreeJoint::ModelTreeJoint(Joint* j, int iJoint) :
	itemJoint(j),
	QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Joint %1").arg(iJoint))) 
{	
	addChild(new QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Name: ")+QString(j->name.c_str()))));	
	
	addChild(new QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Length: %1").arg(j->length))));				
	
	addChild(new QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Connector: ")+QString(j->connector->name.c_str()))));								
	
	positionItem = new QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Position X: %1, Y: %2, Z: %3")
			.arg(j->globalSysV(0), 0, 'g', 2)
			.arg(j->globalSysV(1), 0, 'g', 2)
			.arg(j->globalSysV(2), 0, 'g', 2)));		
	
	addChild(positionItem);
}
	
void ModelTreeJoint::update(void) 
{
	positionItem->setText(0, QString("Position X: %1, Y: %2, Z: %3")
		.arg(itemJoint->globalSysV(0), 0, 'g', 2)
		.arg(itemJoint->globalSysV(1), 0, 'g', 2)
		.arg(itemJoint->globalSysV(2), 0, 'g', 2));
}

ModelTreeLink::ModelTreeLink(Link* l, int iLink) : 
	QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Link %1").arg(iLink))),
	itemLink(l)
{
	addChild(new QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Name: ")+QString(l->name.c_str()))));
		
	addChild(new QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Sensor ID: %1").arg(l->sensorId))));
	
	addChild(new QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Connector: ")+QString(l->connector->name.c_str()))));
		
	coronalAngleItem = new QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Sagittal plane angle: %1").arg(l->planeAngle(0))));
	transverseAngleItem = new QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Transverse plane angle: %1").arg(l->planeAngle(1))));
	sagittalAngleItem = new QTreeWidgetItem((QTreeWidget*) 0, 
		QStringList(QString("Coronal plane angle: %1").arg(l->planeAngle(2))));
		
	addChild(coronalAngleItem);				
	addChild(transverseAngleItem);	
	addChild(sagittalAngleItem);	
}
	
void ModelTreeLink::update(void) 
{
	coronalAngleItem->setText(0, 
		QString("Sagittal plane angle: %1").arg(itemLink->planeAngle(0)));
	transverseAngleItem->setText(0, 
		QString("Transverse plane angle: %1").arg(itemLink->planeAngle(1)));
	sagittalAngleItem->setText(0, 
		QString("Coronal plane angle: %1").arg(itemLink->planeAngle(2)));
}