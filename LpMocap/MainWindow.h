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

#ifndef MAIN_WINDOW
#define MAIN_WINDOW

#include <QWidget>
#include <QMainWindow>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QLabel>
#include <QMessageBox>
#include <QFileDialog>
#include <QPushButton>
#include <QHBoxLayout>
#include <QSplitter>
#include <QToolbar>
#include <QMenu>
#include <QMenuBar>
#include <QLineEdit>
#include <QFileInfo>

#include <thread>
#include <mutex>
#include <iostream>
#include <limits>
using namespace std;

#include "ImuData.h"
#include "HumanModelWindow.h"
#include "PlayControl.h"
#include "GraphWindow.h"
#include "LinkJoint.h"
#include "LPMSRotationData.h"
#include "MotionBuilderSocketInterface.h"

#include <winsock.h>

#include <boost/shared_ptr.hpp>
using namespace boost;

// Representation of human model joint in QT tree
class ModelTreeJoint : public QTreeWidgetItem 
{
public:
	QTreeWidgetItem* positionItem;
	Joint *itemJoint;

	// Constructor
	ModelTreeJoint(Joint* j, int iJoint);
	
	// Updates displayed joint information
	void update(void);
};

// Representation of human model link in QT tree
class ModelTreeLink : public QTreeWidgetItem 
{
public:
	std::string link_name_;
	int link_id_;
	int sensor_id_;
	int parent_link_id_;
	HumanModel *human_model_;
	
	QTreeWidgetItem* coronalAngleItem;
	QTreeWidgetItem* transverseAngleItem;
	QTreeWidgetItem* sagittalAngleItem;

	// Constructor
	ModelTreeLink(int link_id, std::string link_name, int sensor_id, int parent_link_id, HumanModel *human_model);
	
	// Updates displayed link information
	void update(void);
};

// Main window
class MainWindow : public QMainWindow
{
Q_OBJECT
public:
	// Constructor
	MainWindow(QWidget *parent = 0);
	
	// Retrieves current playback timestamp
	QString getPlaybackTimestamp(void);
	
	// Retrieves current recording timestamp
	QString getRecordingTimestamp(void);
	
	// Destructor
	~MainWindow();
	
	void closeConnection(void);
	bool connectToHost(int PortNo, char* IPAddress);
	void initializeSensors(void);
	
public slots:
	void updateImuData(void);	
	void UpdateWindow(void);
	void exitWindow(void);
	void treeItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous);
	void UpdateHumanModel(void);
	void createModelTree(void);
	void ConnectServer(void);
	void DisconnectServer(void);
	void BrowseRecordFile(void);
	void StartRecording(void);
	void ExportToCsv(void);
	void StartPlayback(void);
	void BrowsePlaybackFile(void);
	void ResetOffset(void);
	void ExportAviOfPlayback(void);

public:
	QToolBar *toolbar;
	QMenu* viewMenu;
	HumanModel hm;
	MotionPlayer *mp;
	HumanModelWindow *hmWin;
	GraphWindow *graphWin;
	QPushButton *connectButton;
	QPushButton *recordButton;
	QPushButton *loadButton;
	QPushButton *playButton;
	QPushButton *pauseButton;
	QPushButton *resetButton;
	QPushButton *exitButton;	
	QTreeWidget *humanModelTree;
	vector<ModelTreeJoint*> treeJointList;
	vector<ModelTreeLink*> treeLinkList;		
	QLabel *record_time_label;
	QLabel *play_time_label;
	QLabel *record_file_label;
	QLabel *play_file_label;
	QLineEdit *record_file_edit;
	Link* selectedLink;
	QLineEdit *ip_address_edit;
	QLineEdit *server_port_edit;
	QLabel *connection_status_label;
	bool playback_file_set;
	bool record_file_set;
	QLineEdit *playback_file_edit;
	QAction* start_rec_action;
	QAction* start_playback_action;
	float current_timestamp;
	bool set_offset_all;
	bool offscreen_recording_started_;

	struct {
		char info[50];
		int numSensors;
	} serverInfo;
	
	bool isGetServerInfo;
	int	mChannelCount;
	int	mPassCount;
	int mSocket;
	char mNetworkAddress[64];
	char versionInfo[50];
	int mNetworkPort;
	int mNumSensors;
	FBTCPIP	mTCP;
	int mFps;
	LPMSRotationData mLpmsRotData;
	string record_filename;
	string playback_filename;
	MicroMeasure global_timer;
	bool real_time_thread_stopped;
};

#endif