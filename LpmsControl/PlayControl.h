/***********************************************************************
** Copyright (C) 2013 LP-Research
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

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <iostream>
#include <string>
#include <queue>
#include <fstream>
#include <thread>
#include <mutex>

#include "MicroMeasure.h"
#include "ImuData.h"
#include "LpMatrix.h"

// Widget for playback controller
class PlayController : QWidget {
public:
	double length;

	// Constructor
	PlayController(QWidget* parent = 0);
	
	// Sets playback length
	void setLength(double l);
};

// Plays and records motion of links / joints
class MotionPlayer : public QObject {
Q_OBJECT
public:
	// Constructor
	MotionPlayer(void /* HumanModel	*hm */);
	
	// Retrieves current playback file
	std::string getPlaybackFile(void);
	
	// Updates joints from data read from file
	bool updateJointsFromData(double t);
	
	// Opens motion data file
	bool readMotionDataFile(std::string fn);
	
	// Plays motion data
	void runPlay(void);
	
	// Retrieves current play time
	double currentPlayTime(void);
	
	// True if a motion is currently played back
	bool isPlaying(void);
	
	ImuData getData(void);

public slots:
	// Plays motion
	void play(void);
	
	// Pauses plaback
	void pause(void);
	
	// Resets playback
	void reset(void);
	
public:
	std::vector<ImuData> dataList;
	double maxTime;
	double fps;
	MicroMeasure frameTimer;
	std::ofstream writeStream;
	bool recordingStarted;
	std::queue<ImuData> writeQueue;	
	unsigned long playPointer;
	bool playStarted;
	double playTimerOffset;
	MicroMeasure playTimer;
	double currentTime;
	std::string playbackFile;
	std::string recordingFile;
	ImuData currentData;
};

#endif