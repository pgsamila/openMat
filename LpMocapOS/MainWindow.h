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
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QLabel>
#include <QMessageBox>
#include <QFileDialog>
#include <QPushButton>
#include <QHBoxLayout>
#include <QSplitter>

#include <iostream>
#include <limits>
using namespace std;

#include "ImuData.h"
#include "IceStormSubscriber.h"
#include "HumanModelWindow.h"
#include "PlayControl.h"
#include "GraphWindow.h"

#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp> 
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
	Link* itemLink;
	QTreeWidgetItem* coronalAngleItem;
	QTreeWidgetItem* transverseAngleItem;
	QTreeWidgetItem* sagittalAngleItem;

	// Constructor
	ModelTreeLink(Link* l, int iLink);
	
	// Updates displayed link information
	void update(void);
};

// Main window
class MainWindow : public QWidget
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
	
public slots:
	// Updates current IMU data
	void updateImuData(ImuData ld);
	
	// Connects to server
	void connectServer(void);
	
	// Updates window
	void updateWindow(void);
	
	// Starts recording
	void recordMotion(void);
	
	// Loads motion data
	void loadMotion(void);
	
	// Plays motions
	void playMotion(void);
	
	// Exits application
	void exitWindow(void);
	
	// Called when tree item is changed
	void treeItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous);
	
	// Exports motion data to CSV file
	void exportCsv(void);
	
	// Resets offset data for initialization
	void resetOffset(void);
	
public:
	IceStormSubscriber *iss;
	HumanModel *hm;
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
	QLabel *recordingTimeLabel;
	QLabel *playbackTimeLabel;
	QLabel *recordingFileLabel;
	QLabel *playbackFileLabel;
	Link* selectedLink;
};

#endif