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

#include "PlayControl.h"

PlayController::PlayController(QWidget* parent) : 
	QWidget(parent) 
{
}

void PlayController::setLength(double l) 
{
	length = l;
}

MotionPlayer::MotionPlayer(void) :
	maxTime(9999),
	fps(30),
	recordingStarted(false),
	playStarted(false),
	playPointer(0),
	playTimerOffset(0),
	currentTime(0),
	playbackFile("n/a"),
	recordingFile("n/a")
{
}

std::string MotionPlayer::getPlaybackFile(void) {
	return playbackFile;
}

bool MotionPlayer::updateJointsFromData(double t) 
{
	ImuData d;
	
	if (playPointer < dataList.size()) {
		d = dataList[playPointer];
	} else {
		reset();
		return false;
	}
	
	if (t > (d.timeStamp * 1000.0f)) {								
		currentTime = d.timeStamp;
		currentData = d;
					
		++playPointer;
	}
		
	return true;
}	

bool MotionPlayer::readMotionDataFile(std::string fn) {
	int i;
	std::string line;
	LpVector4f q;
	LpMatrix3x3f m;
	
	std::ifstream dataFile(fn.c_str(), std::ios::in);	

	dataList.clear();

	if (dataFile.is_open()) {
		getline(dataFile, line); // Header
	
		while (dataFile.good()) {				
			ImuData d;
			
			getline(dataFile, line);
			
			boost::char_separator<char> sep(", ");
			boost::tokenizer<boost::char_separator<char>> tokens(line, sep);
			boost::tokenizer<boost::char_separator<char>>::iterator ti;
			
			if (tokens.begin() == tokens.end()) break;
			
			// SensorId, TimeStamp (s), FrameNumber, AccX (g), AccY (g), AccZ (g), GyroX (deg/s), GyroY (deg/s), GyroZ (deg/s), MagX (uT), MagY (uT), MagZ (uT), EulerX (deg), EulerY (deg), EulerZ (deg), QuatX, QuatY, QuatZ, QuatW, LinAccX (m/s^2), LinAccY (m/s^2), LinAccZ (m/s^2), Pressure (kPa), Altitude (m), Temperature (degC), HeaveMotion (m)			
			
			ti = tokens.begin();
			d.openMatId = boost::lexical_cast<double>(*ti); ++ti;
			d.timeStamp = boost::lexical_cast<double>(*ti); ++ti;
			d.frameCount = boost::lexical_cast<double>(*ti); ++ti;			
			for (i=0; i<3; ++i) {
				d.a[i] = boost::lexical_cast<double>(*ti); ++ti;
			}
			for (i=0; i<3; ++i) {
				d.g[i] = boost::lexical_cast<double>(*ti); ++ti;
			}
			for (i=0; i<3; ++i) {
				d.b[i] = boost::lexical_cast<double>(*ti); ++ti;
			}
			for (i=0; i<3; ++i) {
				d.r[i] = boost::lexical_cast<double>(*ti); ++ti;
			}
			for (i=0; i<4; ++i) {
				d.q[i] = boost::lexical_cast<double>(*ti); ++ti;
			}
			for (i=0; i<4; ++i) {
				d.linAcc[i] = boost::lexical_cast<double>(*ti); ++ti;
			}
			
			convertArrayToLpVector4f(d.q, &q);			
			quaternionToMatrix(&q, &m);
			convertLpMatrixToArray(&m, d.rotationM);				
			
			// printf("[PlayControl] Read t=%f, q0=%f, q1=%f, q2=%f, q3=%f\n", d.timeStamp, d.q[0], d.q[1], d.q[2], d.q[3]);
			
			dataList.push_back(d);
			maxTime = d.timeStamp;				
		}
		dataFile.close();
	
		playStarted = false;		
		reset();
		play();
		
		playbackFile = fn;
		std::cout << "[PlayControl] Playing motion data from " << fn.c_str() << std::endl;			
	} else {
		std::cout << "[PlayControl] Could not open file " << fn << std::endl;
		return false;
	}
	return true;
}

void MotionPlayer::runPlay(void)
{		
	playTimer.reset();
	while (playStarted == true) {
		updateJointsFromData((double) playTimer.measure() / 1000.0 + playTimerOffset);
		std::this_thread::sleep_for(std::chrono::microseconds(1000));
	}
}

double MotionPlayer::currentPlayTime(void)
{
	return currentTime;
}

bool MotionPlayer::isPlaying(void)
{
	return playStarted;
}

void MotionPlayer::play(void)
{
	if (playStarted == false) {		
		playStarted = true;		
		std::thread t(&MotionPlayer::runPlay, this);
		t.detach();
	}
}

void MotionPlayer::pause(void)
{
	if (playStarted == true) {
		playTimerOffset = (double) playTimer.measure() / 1000.0 + playTimerOffset;		
		playStarted = false;
	}
}

void MotionPlayer::reset(void)
{
	playPointer = 0;
	playTimerOffset = 0;
	playTimer.reset();
	play();
}

ImuData MotionPlayer::getData(void)
{
	return currentData;
}