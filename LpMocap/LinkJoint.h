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

#ifndef LINK_JOINT
#define LINK_JOINT

#include <string>
#include <iostream>
using namespace std;

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

// Contains all information about a joint
class Joint 
{
public:
	// Constructor
	Joint(	Eigen::Vector4f baseV,
			Eigen::Vector3f rotV,
			float length,
			Joint* connector,
			string name);

public:
	// Joint base vector
	Eigen::Vector4f baseV;
	
	// Joint default rotation as defined in model
	Eigen::Vector3f rotV;
	
	// Joint length
	float length;
	
	// Where the joint is coming from
	Joint* connector;
	
	// Joint name
	string name;
	
	// Vector to the base of the joint in its local (link) coordinate system
	Eigen::Vector4f localSysV;
	
	// Vector to the base of the joint in global cordinate system
	Eigen::Vector4f globalSysV;
	
	// Rotation / translation matrix of a link
	Eigen::Matrix4f linkRotM;
	
	// Rotation / translation matrix in global coordinate system
	Eigen::Matrix4f globalT;
	
	// Rotation/ translation matrix of joint in local coordinate system
	Eigen::Matrix4f localT;	
};

// Joint colors
enum COLORS {
	blue,
	red,
	green
};

// Contains link information
class Link 
{
public:
	// Constructor
	Link(	Joint *connector,
			int sensorId,
			enum COLORS color,
			string name,
			vector<Joint*> jointList,
			Eigen::Matrix4f rotScaleM,
			Eigen::Vector3f sensorOrientation);

public:
	// Where this link is coming from
	Joint *connector;
	
	// ID of the sensor associated to the link
	int sensorId;
	
	// Name of the link
	string name;
	
	// List of joints in the link
	vector<Joint*> jointList;

	// Rotation / translation matrix of the link
	Eigen::Matrix4f linkRotM;
	
	// Indicates if link offset has been set (model initialization)
	bool offsetSet;
	
	// Sagittal, coronal and transversal angles
	Eigen::Vector3f planeAngle;
	
	// Color of the link
	enum COLORS color;
	
	// Deactivation timeout
	unsigned long timeout;
	
	// Indicates if link is active
	bool active;
	
	// Orientation of the sensor relative to the joint
	Eigen::Vector3f sensorFixRotV;
	
	// Offset rotation matrix
	Eigen::Matrix4f rotOffsetM;
	
	// Scaling matrix for the link rotation
	Eigen::Matrix4f rotScaleM;
	
	// Raw rottaion matrix from sensor rotation quaternion
	Eigen::Matrix4f rawRotM;
	
	// Sensor orientation relative to joint in Euler angles
	Eigen::Vector3f sensorOrientation;
};

#endif