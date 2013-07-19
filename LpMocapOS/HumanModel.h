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

#ifndef HUMAN_MODEL
#define HUMAN_MODEL

#include <QDomNode>
#include <QDomElement>
#include <QDomDocument>
#include <QFile>
#include <QTimer>

#include <string>
#include <iostream>
using namespace std;

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
using namespace boost;

#include <Eigen/Dense>

#include "ImuData.h"
#include "LinkJoint.h"

// Contains routines to calculate joint positions from joint / IMU angles
class ForwardKinematics
{
private:
	// List of links (connections between joints)
	vector<Link*> linkList;
	
	// List of joints (connections between joints)
	vector<Joint*> jointList;
	
public:
	// Constructor
	ForwardKinematics(vector<Link*> linkList, vector<Joint*> jointList);
	
	// Calculates new joint positions from joint angles
	void update(void);	
	
	// Calcualtes sagittal, transversal and coronal plane angles
	Eigen::Vector3f projectionAngle(Eigen::Matrix3f rm);
};

// Contains human model data and calculates joint positions etc. in model from IMU data
class HumanModel : public QObject 
{
Q_OBJECT
public:
	// Constructor
	HumanModel(void);
	
	// Reads model XML data
	void readModel(string fn);
	
	// Calculates joint positions from joint angles
	void recalculateJoints(void);
	
	// Converts rotation quaternion to rotation matrix
	Eigen::Matrix3f qToM(float* q);
	
	// Converts quaternion to 4D rotation / translation atrix
	Eigen::Matrix4f quat2Matrix(Eigen::Vector4f q);
	
	// Converts quaternion to Euler angles
	Eigen::Vector3f quat2Euler(Eigen::Vector4f q);
	
	// Initializes sensors
	void initializeSensors(void);
	
	// Checks if joint data has not been updated for timeout time
	void checkTimeout(void);
	
	// Resets offset value for all joints. Model initialization.
	void resetOffset(void);
	
public slots:
	// Updates object with new sensor data
	void updateSensorData(ImuData imuData);
	
public:
	// Calculates forward kinematics
	ForwardKinematics* fk;
	
	// List of joints (pointers)
	vector<Joint*> jointList;
	
	// List of links (pointers)
	vector<Link*> linkList;
};

#endif