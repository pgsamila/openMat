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

#include "PlayControl.h"

std::mutex recordMutex;

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
	JointDataSet d;
	
	if (playPointer < dataList.size()) {
		d = dataList[playPointer];
	} else {
		reset();
		return false;
	}
	
	if (t > d.timeStamp) {			
		vector<Link*>::iterator i = linkList.begin();
		BOOST_FOREACH(Eigen::Matrix3f mat, d.linkRotationData) {
			(*i)->linkRotM.block<3, 3>(0, 0) = mat;
			++i;
		}
					
		currentTime = d.timeStamp;		
					
		++playPointer;
	}
	
	BOOST_FOREACH(Link* l, linkList) {	
		l->active = true;
	}
	
	return true;
}	

bool MotionPlayer::readMotionDataFile(std::string fn) {
	std::string line;
	double x;
	int i;
	int mSet;
	Eigen::Vector3f v;	
	std::ifstream dataFile(fn.c_str(), std::ios::in | std::ios::binary);	

	dataList.clear();

	if (dataFile.is_open()) {
		while (dataFile.good()) {				
			JointDataSet d;
			getline(dataFile, line);
			boost::char_separator<char> sep(", ");
			boost::tokenizer<boost::char_separator<char>> tokens(line, sep);
			boost::tokenizer<boost::char_separator<char>>::iterator ti;
			
			if (tokens.begin() == tokens.end()) break;
			
			ti = tokens.begin();
			d.timeStamp = boost::lexical_cast<double>(*ti);
			i = 0;
			++ti;
							
			for (; ti != tokens.end(); ++ti) {
				Eigen::Matrix3f mat;				
				x = boost::lexical_cast<double>(*ti);
				mSet = i%9;
				mat(mSet/3, mSet%3) = x;
				if (mSet%3 == 2 && mSet/3 == 2) d.linkRotationData.push_back(mat);
				++i;
			}				
			dataList.push_back(d);
			maxTime = d.timeStamp;				
		}
		dataFile.close();
	
		playStarted = false;		
		reset();
		play();
		
		playbackFile = fn;
		cout << "[Motion data player] Playing motion data from " << fn.c_str() << std::endl;			
	} else {
		std::cout << "[Motion data player] Could not open file " << fn << endl;
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

bool MotionPlayer::recordMotionDataFile(std::string fn)
{
	writeStream.open(fn.c_str(), std::ios::out | std::ios::binary);
	frameTimer.reset();
	
	if (writeStream.is_open()) {
		recordingStarted = true;		
		
		std::thread t(&MotionPlayer::runRecording, this);
		t.detach();	

		std::cout << "[Motion data recorder] Writing motion data to " << fn.c_str() << std::endl;	
		
		recordingFile = fn;			
		return true;
	}

	std::cout << "[Motion data recorder] Failed to open " << fn.c_str() << std::endl;		

	return false;
}

bool MotionPlayer::isRecording(void)
{
	return recordingStarted;
}

void MotionPlayer::stopRecording(void)
{
	recordingStarted = false;
}
	
void MotionPlayer::runRecording(void)
{		
	while (writeQueue.empty() == false) {
		writeQueue.pop();
	}	

	while (recordingStarted == true || writeQueue.empty() == false) {
		recordMutex.lock();
		writeData();
		recordMutex.unlock();
	}
	
	writeStream.close();
}

void MotionPlayer::writeData(void)
{
	JointDataSet latestData;

	while (writeQueue.empty() == false)
	{
		latestData = writeQueue.front();
		writeQueue.pop();
						
		if (writeStream.is_open())
		{				
			writeStream << latestData.timeStamp;
					
			BOOST_FOREACH(Eigen::Matrix3f mat, latestData.linkRotationData) {
				writeStream << "," << mat(0, 0) << "," << mat(0, 1) << "," << mat(0, 2) <<
					"," << mat(1, 0) << "," << mat(1, 1) << "," << mat(1, 2) <<
					"," << mat(2, 0) << "," << mat(2, 1) << "," << mat(2, 2);
			}
			writeStream << std::endl;
		}
	}
}

bool MotionPlayer::writeCSVData(std::string fn)
{
	JointDataSet d;
	ofstream csvStream;
	
	csvStream.open(fn.c_str());	
	
	if (csvStream.is_open() == false) return false;	
	
	csvStream << "OpenMAT human model csv data output file" << std::endl;
	csvStream << "TS: Timestamp (s)" << std::endl;
	csvStream << "LN: Link name" << std::endl;
	csvStream << "SP: Sagittal plane angle (degree)" << std::endl;
	csvStream << "TP: Transverse plane angle (degree)" << std::endl;
	csvStream << "CP: Coronal plane angle (degree)" << std::endl;
	csvStream << "JN: Joint name" << std::endl;	
	csvStream << "XP: Joint x-axis position (rel. units)" << std::endl;	
	csvStream << "YP: Joint y-axis position (rel. units)" << std::endl;	
	csvStream << "ZP: Joint z-axis position (rel. units)" << std::endl << std::endl;	
	
	csvStream << "TS";		
	BOOST_FOREACH(Link* l, linkList) {
		csvStream << ", LN: " << l->name;
		csvStream << " SP, TP, CP";
			
		BOOST_FOREACH(Joint* j, l->jointList) {
			csvStream << ", JN: " << j->name;
			csvStream << " XP, YP, ZP";
		}
	}
	csvStream << std::endl;
	
	for (unsigned int j=0; j<dataList.size(); ++j) {
		d = dataList[j];
		
		vector<Link*>::iterator i = linkList.begin();
		BOOST_FOREACH(Eigen::Matrix3f mat, d.linkRotationData) {
			(*i)->linkRotM.block<3, 3>(0, 0) = mat;
			++i;
		}

		csvStream << d.timeStamp;		
		BOOST_FOREACH(Link* l, linkList) {
			csvStream << ", " << l->planeAngle(0) << 
				", " << l->planeAngle(1) <<
				", " << l->planeAngle(2);

			BOOST_FOREACH(Joint* j, l->jointList) {
				csvStream << ", " << j->globalSysV(0) << 
					", " << j->globalSysV(1) <<
					", " << j->globalSysV(1);
			}
		}
		
		csvStream << std::endl;
	}
	
	writeStream.close();	
	
	return true;
}

double MotionPlayer::currentRecordTime(void)
{
	return (double) frameTimer.measure() / 1000.0;
}	

std::string MotionPlayer::getRecordingFile(void) {
	return recordingFile;
}	

void MotionPlayer::newMotionData(void)
{
	if (recordingStarted == false)
	{
		return;
	}
	
	JointDataSet jSet;
	
	recordMutex.lock();
	
	BOOST_FOREACH(Link *l, linkList) {
		jSet.linkRotationData.push_back(l->linkRotM.block<3, 3>(0, 0));
	}
	
	jSet.timeStamp = (double) frameTimer.measure() / 1000.0;
	writeQueue.push(jSet);
	
	recordMutex.unlock();
}