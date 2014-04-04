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
	mp(new MotionPlayer()),
	hmWin(new HumanModelWindow(&hm)),
	graphWin(new GraphWindow()),
	humanModelTree(new QTreeWidget())
{	
	int i = 0;

	toolbar = new QToolBar("Toolbar");
	addToolBar(Qt::TopToolBarArea, toolbar);

	toolbar->setMovable(false);
	toolbar->setFloatable(false);

// Server IP address toolbar
	QMenu* connectMenu = menuBar()->addMenu("&Connect");
	
	QAction* connect_action = new QAction(QIcon("./icons/bolt_32x32.png"), "&Connect", this);
	QAction* disconnect_action = new QAction(QIcon("./icons/x_28x28.png"), "&Disconnect", this);
	QAction* exitAction = new QAction("E&xit program", this);
	
	QVBoxLayout *ip_address_layout = new QVBoxLayout();
	ip_address_edit = new QLineEdit();
	ip_address_edit->setText("192.168.7.2");
	ip_address_layout->addWidget(new QLabel("Server IP address:"));
	ip_address_layout->addWidget(ip_address_edit);
	QWidget *ip_address_widget = new QWidget();
	ip_address_widget->setLayout(ip_address_layout);
	ip_address_widget->setFixedWidth(150);
	
	QVBoxLayout *server_port_layout = new QVBoxLayout();
	server_port_edit = new QLineEdit();
	server_port_edit->setText("8889");
	server_port_layout->addWidget(new QLabel("Port:"));
	server_port_layout->addWidget(server_port_edit);
	QWidget *server_port_widget = new QWidget();
	server_port_widget->setLayout(server_port_layout);
	server_port_widget->setFixedWidth(75);

	QVBoxLayout *connection_status_layout = new QVBoxLayout();
	connection_status_layout->addWidget(new QLabel("Connection status:"), i, 0);
	connection_status_label = new QLabel("<font color='red'>DISCONNECTED</font>");
	connection_status_layout->addWidget(connection_status_label);
	QWidget *connection_status_widget = new QWidget();
	connection_status_widget->setLayout(connection_status_layout);
	connection_status_widget->setFixedWidth(120);

	connectMenu->addAction(connect_action);
	connectMenu->addAction(disconnect_action);
	connectMenu->addSeparator();	
	connectMenu->addAction(exitAction);	

	toolbar->addWidget(ip_address_widget);
	toolbar->addWidget(server_port_widget);
	toolbar->addAction(connect_action);
	toolbar->addAction(disconnect_action);
	toolbar->addWidget(connection_status_widget);
	
// Recording toolbar
	QMenu* measurementMenu = menuBar()->addMenu("&Measurement");

	QAction* browse_rec_file_action = new QAction(QIcon("./icons/folder_stroke_32x32.png"), "&Browse record file", this);	
	start_rec_action = new QAction(QIcon("./icons/layers_32x28.png"), "&Record data", this);
	QAction* export_csv_action = new QAction(QIcon("./icons/loop_alt2_32x28.png"), "&Export CSV file", this);

	measurementMenu->addAction(browse_rec_file_action);
	measurementMenu->addAction(start_rec_action);

	toolbar->addSeparator();
	QVBoxLayout *v = new QVBoxLayout();
	record_file_edit = new QLineEdit();
	record_file_edit->setReadOnly(true);	
	record_file_edit->setText("Not set, please browse..");
	record_file_set = false;
	v->addWidget(new QLabel("Record filename:"));
	v->addWidget(record_file_edit);
	QWidget *w = new QWidget();
	w->setLayout(v);
	w->setFixedWidth(200);

	QVBoxLayout *rec_time_layout = new QVBoxLayout();
	rec_time_layout->addWidget(new QLabel("Recording time:"), i, 0);
	record_time_label = new QLabel("<font color='red'>0h 0m 0s 0ms</font>");
	rec_time_layout->addWidget(record_time_label);
	QWidget *rec_time_widget = new QWidget();
	rec_time_widget->setLayout(rec_time_layout);
	rec_time_widget->setFixedWidth(100);

	toolbar->addAction(start_rec_action);
	toolbar->addWidget(w);
	toolbar->addAction(browse_rec_file_action);
	toolbar->addWidget(rec_time_widget);
	// toolbar->addAction(export_csv_action);
	
// Playback toolbar
	start_playback_action = new QAction(QIcon("./icons/play_24x32.png"), "&Playback data", this);
	QAction* browse_playback_file_action = new QAction(QIcon("./icons/folder_stroke_32x32.png"), "&Browse replay file", this);
	
	toolbar->addSeparator();
	QVBoxLayout *v4 = new QVBoxLayout();
	playback_file_edit = new QLineEdit();
	playback_file_edit->setReadOnly(true);	
	playback_file_edit->setText("Not set, please browse..");
	playback_file_set = false;
	v4->addWidget(new QLabel("Playback filename:"));
	v4->addWidget(playback_file_edit);
	QWidget *w4 = new QWidget();
	w4->setLayout(v4);
	w4->setFixedWidth(200);

	QVBoxLayout *play_time_layout = new QVBoxLayout();
	play_time_layout->addWidget(new QLabel("Playback time:"), i, 0);
	play_time_label = new QLabel("<font color='green'>0h 0m 0s 0ms</font>");
	play_time_layout->addWidget(play_time_label);
	QWidget *play_time_widget = new QWidget();
	play_time_widget->setLayout(play_time_layout);
	play_time_widget->setFixedWidth(100);

	toolbar->addAction(start_playback_action);	
	toolbar->addWidget(w4);
	toolbar->addAction(browse_playback_file_action);	
	toolbar->addWidget(play_time_widget);

	measurementMenu->addSeparator();
	measurementMenu->addAction(start_playback_action);
	measurementMenu->addAction(browse_playback_file_action);

	QAction* reset_offset_action = new QAction(QIcon("./icons/fullscreen_exit_32x32.png"), "Rese&t offset", this);

	toolbar->addSeparator();
	toolbar->addAction(reset_offset_action);
	toolbar->addSeparator();
	
	measurementMenu->addSeparator();
	measurementMenu->addAction(reset_offset_action);

	QVBoxLayout *v0 = new QVBoxLayout();
	v0->addWidget(humanModelTree);
		
	setCentralWidget(hmWin);
	
	connect(connect_action, SIGNAL(triggered()), this, SLOT(ConnectServer()));
	connect(disconnect_action, SIGNAL(triggered()), this, SLOT(DisconnectServer()));

	connect(browse_rec_file_action, SIGNAL(triggered()), this, SLOT(BrowseRecordFile()));
	connect(start_rec_action, SIGNAL(triggered()), this, SLOT(StartRecording()));
	connect(export_csv_action, SIGNAL(triggered()), this, SLOT(ExportToCsv()));

	connect(start_playback_action, SIGNAL(triggered()), this, SLOT(StartPlayback()));
	connect(browse_playback_file_action, SIGNAL(triggered()), this, SLOT(BrowsePlaybackFile()));

	connect(reset_offset_action, SIGNAL(triggered()), this, SLOT(ResetOffset()));
	
	hm.loadHumanModel("HumanModel.xml");

	createModelTree();
	
	hmWin->updateGL();

	mNetworkPort = 8889;
	strcpy(mNetworkAddress, "192.168.7.2");

	current_timestamp = 0;
	set_offset_all = false;

	QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateWindow()));
    timer->start(1);
	
	global_timer.reset();

	showMaximized();
}

void MainWindow::ConnectServer(void)
{
	mNetworkPort = server_port_edit->text().toInt();
	strcpy(mNetworkAddress, ip_address_edit->text().toStdString().c_str());
	if (mTCP.CreateSocket(mSocket, kFBTCPIP_Stream)) {
		if (mTCP.Connect(mSocket, mNetworkAddress, mNetworkPort) == false) {
			std::cout << "[LpMocap] Connection to server failed." << std::endl;
			if (mSocket) mTCP.CloseSocket(mSocket);
			mSocket = 0;
			connection_status_label->setText("<font color='red'>DISCONNECTED</font>");
		} else {
			std::cout << "[LpMocap] Connected to server." << std::endl; 
				 
			int tmpByteCount;

			mTCP.Write(mSocket, (char*)&LPMB_GET_INFO, sizeof(LPMB_GET_INFO)); 
			mTCP.ReadBlocking(mSocket, (char*)&serverInfo, sizeof(serverInfo), &tmpByteCount );
				
			strcpy_s(versionInfo, serverInfo.info);
				
			mNumSensors =  serverInfo.numSensors;

			connection_status_label->setText("<font color='green'>CONNECTED</font>");
		}
	}
}

void MainWindow::DisconnectServer(void)
{
	mTCP.CloseSocket(mSocket);
	connection_status_label->setText("<font color='red'>DISCONNECTED</font>");
}

void MainWindow::BrowseRecordFile(void)
{
	std::string fn;
	QString qfilename = QFileDialog::getSaveFileName(this, "Save motion data", "./", "LpMocap motion data file (*.omd)");
	fn = qfilename.toStdString();
	if (fn != "") {
		record_filename = fn;
		record_file_edit->setText(qfilename.toStdString().c_str());
		record_file_set = true;
	}
}

void MainWindow::StartRecording(void)
{
	if (hm.IsRecordOn() == false) {
		if (record_file_set == true) {
			printf("[LpMocap] Starting recording to %s\n", record_filename.c_str());
			hm.StartSaveBinaryMotionData(record_filename.c_str());
			start_rec_action->setText("Stop recording");
			start_rec_action->setIcon(QIcon("./icons/x_alt_32x32.png"));
		}
	} else {
		hm.StopSaveBinaryMotionData();
		start_rec_action->setIcon(QIcon("./icons/layers_32x28.png"));
		start_rec_action->setText("Record data");
	}
}

void MainWindow::ExportToCsv(void)
{
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

void MainWindow::StartPlayback(void)
{
	if (hm.IsPlaybackOn() == false) {
		if (playback_file_set == true) {
			printf("[LpMocap] Starting playback of %s\n", playback_filename.c_str());
			hm.StartPlayBinaryMotionData(playback_filename.c_str());
			start_playback_action->setText("Stop playback");
			start_playback_action->setIcon(QIcon("./icons/x_alt_32x32.png"));
		}
	} else {
		hm.StopPlayBinaryMotionData();
		start_playback_action->setIcon(QIcon("./icons/play_24x32.png"));
		start_playback_action->setText("Playback data");
	}
}

void MainWindow::BrowsePlaybackFile(void)
{
	std::string fn;
	QString qfilename = QFileDialog::getOpenFileName(this, "Load motion data", "./", "OpenMAT motion data file (*.omd);;Motion Builder C3D (*.c3d)");	
	
	fn = qfilename.toStdString();
	
	QFileInfo file_info(qfilename);
	QString file_extension = file_info.suffix();
			
	if (fn != "") {
		playback_filename = fn;
		playback_file_edit->setText(qfilename.toStdString().c_str());
		playback_file_set = true;
		if (file_extension == "c3d") {
			hm.ReadC3dFile(qfilename.toStdString().c_str());		
		} else {
			hm.ReadBinaryMotionDataFile(qfilename.toStdString().c_str());
		}
	}
}

void MainWindow::ResetOffset(void)
{
	initializeSensors();

	// set_offset_all = true;
}

void MainWindow::createModelTree(void)
{
	humanModelTree->setColumnCount(1);
	humanModelTree->setHeaderLabel(QString("Human model"));	
	QTreeWidgetItem *modelRoot = new QTreeWidgetItem((QTreeWidget*) 0, QStringList(QString("Model root")));	
	humanModelTree->insertTopLevelItem(0, modelRoot);

	for (int i=0; i < hm.mChannelCount; ++i) {
		ModelTreeLink *newLink = new ModelTreeLink(i, std::string(hm.GetChannelName(i)), i, hm.GetChannelParent(i), &hm);
		newLink->setText(0, hm.GetChannelName(i));

		modelRoot->addChild(newLink);
		treeLinkList.push_back(newLink);		
	}

	modelRoot->setExpanded(true);
	humanModelTree->setCurrentItem((QTreeWidgetItem *)treeLinkList[0]);
}

QString MainWindow::getPlaybackTimestamp(void)
{
	QString ts;
	
	double t = hm.GetPlaybackTime();
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

QString MainWindow::getRecordingTimestamp(void)
{
	QString ts;
	
	double t = hm.GetRecordTime();
	long h = (long) t / 1000.0 / 60.0 / 60.0;
	long m = (long)		(t 	- ((double) h * 60.0 * 60.0 * 1000.0)) / 1000.0 / 60.0;
	long s = 			(t 	- ((double) h * 60.0 * 60.0 * 1000.0) 
							- ((double) m * 60.0 * 1000.0)) / 1000;
	long ms = (long)	(t 	- ((double) h * 60.0 * 60.0 * 1000.0) 
							- ((double) m * 60.0 * 1000.0) 
							- ((double) s * 1000.0));
			
	ts = QString("<font color='red'>%1h %2m %3s %4ms</font>").arg(h).arg(m).arg(s).arg(ms);
	return ts;
}

void MainWindow::updateWindow(void)
{	
	current_timestamp += (float) global_timer.measure() / 1000.0f;
	global_timer.reset();

	hm.UpdateTimestamp(current_timestamp);

	updateImuData();

	hm.UpdateSaveBinaryMotionData();
	hm.UpdateModelFromData();

	play_time_label->setText(getPlaybackTimestamp());
	record_time_label->setText(getRecordingTimestamp());
}
	
MainWindow::~MainWindow()
{
}

void MainWindow::initializeSensors(void)
{
	hm.resetOffsetAll();

	if (mSocket) {
		for (int i=0; i != mLpmsRotData.ChannelCount; ++i){
			int sensorId = mLpmsRotData.mChannel[i].id;
			
			if (sensorId > 0 && sensorId <= hm.mChannelCount){
				hm.setOffset(sensorId-1, mLpmsRotData.mChannel[i].q);			 
			} 
		}
	}
	
	hm.resetSkeleton();
}

void MainWindow::treeItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous)
{
}

void MainWindow::updateImuData(void)
{
	if (mSocket && hm.IsPlaybackOn() == false) {					
		int tmpByteCount = 0;

		mTCP.Write(mSocket, (char*)&LPMB_GET_DATA, sizeof(LPMB_GET_DATA));
		
		if (mTCP.ReadBlocking(mSocket, (char*)&mLpmsRotData, sizeof(mLpmsRotData), &tmpByteCount)) {
			for (int i=0; i != mLpmsRotData.ChannelCount; ++i) {
				if (set_offset_all == true) {
					int sensorId = mLpmsRotData.mChannel[i].id;
					
					if (sensorId > 0 && sensorId <= hm.mChannelCount){
						hm.setOffset(sensorId-1, mLpmsRotData.mChannel[i].q);			 
					} 
				}
				hm.decodeSensorRotation(mLpmsRotData.mChannel[i].id, mLpmsRotData.mChannel[i].q);
			}
		} else {
			return;
		}		

		if (set_offset_all) hm.resetSkeleton();
		
		hm.updateBodyData();

		set_offset_all = false;
	}

	hmWin->updateGL();
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

ModelTreeLink::ModelTreeLink(int link_id, std::string link_name, int sensor_id, int parent_link_id, HumanModel *human_model) : 
	QTreeWidgetItem((QTreeWidget*) 0), 
	link_id_(link_id),
	link_name_(link_name),
	sensor_id_(sensor_id),
	parent_link_id_(parent_link_id),
	human_model_(human_model)
{
	addChild(new QTreeWidgetItem((QTreeWidget*) 0, QStringList(QString("Link ID: %1").arg(link_id_))));
	addChild(new QTreeWidgetItem((QTreeWidget*) 0, QStringList(QString("Sensor ID: %1").arg(sensor_id_))));
	addChild(new QTreeWidgetItem((QTreeWidget*) 0, QStringList(QString("Parent link: %1").arg(parent_link_id_))));
		
	coronalAngleItem = new QTreeWidgetItem((QTreeWidget*) 0, QStringList(QString("Sagittal plane angle: %1").arg(human_model->getProjectionAngle(link_id_)(0))));
	
	transverseAngleItem = new QTreeWidgetItem((QTreeWidget*) 0, QStringList(QString("Transverse plane angle: %1").arg(human_model->getProjectionAngle(link_id_)(1))));
	
	sagittalAngleItem = new QTreeWidgetItem((QTreeWidget*) 0, QStringList(QString("Coronal plane angle: %1").arg(human_model->getProjectionAngle(link_id_)(2))));
	
	addChild(coronalAngleItem);		
	addChild(transverseAngleItem);
	addChild(sagittalAngleItem);
}
	
void ModelTreeLink::update(void) 
{
	coronalAngleItem->setText(0, QString("Sagittal plane angle: %1").arg(human_model_->getProjectionAngle(link_id_)(0)));
	transverseAngleItem->setText(0, QString("Transverse plane angle: %1").arg(human_model_->getProjectionAngle(link_id_)(1)));
	sagittalAngleItem->setText(0, QString("Coronal plane angle: %1").arg(human_model_->getProjectionAngle(link_id_)(2)));
}