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

#ifndef PLAY_CONTROL
#define PLAY_CONTROL

#include <QWidget>

#include <boost/thread/thread.hpp> 
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
using namespace boost;

#include <iostream>
#include <string>
#include <queue>
#include <fstream>
using namespace std;

#include "MicroMeasure.h"

#include "HumanModel.h"

// Widget for playback controller
class PlayController : QWidget {
public:
	double length;

	// Constructor
	PlayController(QWidget* parent = 0);
	
	// Sets playback length
	void setLength(double l);
};

// One data set of all links and joints for the model
class JointDataSet {
public:
	double timeStamp;
	vector<Eigen::Vector3f> jointPositionData;
	vector<Eigen::Matrix3f> linkRotationData;
};

// Plays and records motion of links / joints
class MotionPlayer : public QObject {
Q_OBJECT
public:
	// Constructor
	MotionPlayer(HumanModel	*hm);
	
	// Retrieves current playback file
	string getPlaybackFile(void);
	
	// Updates joints from data read from file
	bool updateJointsFromData(double t);
	
	// Opens motion data file
	bool readMotionDataFile(string fn);
	
	// Plays motion data
	void runPlay(void);
	
	// Retrieves current play time
	double currentPlayTime(void);
	
	// True if a motion is currently played back
	bool isPlaying(void);
	
	// Records motion data to a file
	bool recordMotionDataFile(string fn);
	
	// True if a motion is currently recorded
	bool isRecording(void);
	
	// Stops recording
	void stopRecording(void);
	
	// Starts recording
	void runRecording(void);
	
	// Writes data to file
	void writeData(void);
	
	// Retrieves current recording time
	double currentRecordTime(void);
	
	// Retrievs current recording file
	string getRecordingFile(void);
	
	// Writes current motion data to CSV file
	bool writeCSVData(string fn);

public slots:
	// Plays motion
	void play(void);
	
	// Pauses plaback
	void pause(void);
	
	// Resets playback
	void reset(void);
	
	// Reinitializes
	void newMotionData(void);
	
public:
	HumanModel *hm;
	vector<Joint*> jointList;
	vector<Link*> linkList;	
	vector<JointDataSet> dataList;
	double maxTime;
	double fps;
	MicroMeasure frameTimer;
	std::ofstream writeStream;
	bool recordingStarted;
	queue<JointDataSet> writeQueue;	
	unsigned long playPointer;
	bool playStarted;
	double playTimerOffset;
	MicroMeasure playTimer;
	double currentTime;
	string playbackFile;
	string recordingFile;
};

#endif