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

#include "HumanModel.h"

HumanModel::HumanModel(void)
{
	readModel("HumanModel.xml");
	
	fk = new ForwardKinematics(linkList, jointList);
	recalculateJoints();
	
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(checkTimeout()));
	timer->start(100);	
}

void HumanModel::readModel(string fn) 
{
	string name;
	int i;
	double x;
	string line;

	QDomDocument doc("HumanModel");
	QFile file(fn.c_str());

	if (!file.open(QIODevice::ReadOnly)) {
		cout << "[Human Model] Could not open configuration file" << endl;
  		return;
	}

	if (!doc.setContent(&file)) {
  		file.close();
		cout << "[Human Model] Could not parse XML code" << endl;
  		return;
	}

	file.close();

	QDomElement root = doc.documentElement();

	if (root.tagName() != "HumanModel") {
		cout << "[Human Model] XML file doesn't contain HumanModel tag" << endl;
  		return;
	}
	
	QDomNode humanModelN = root.firstChild();
	while (!humanModelN.isNull()) {
		QDomElement humanModelE = humanModelN.toElement(); 
		if (humanModelE.tagName() == "Link") {	
			cout << "[Human Model] Initializing new link" << endl;		
			
			QDomNode linkN = humanModelN.firstChild();
			vector<Joint*> lJointList;
			string lName;
			enum COLORS lColor;
			int lSensorId;
			Joint *lConnector;
			Eigen::Vector3f sensorOrientation(Eigen::Vector3f::Zero());			
			Eigen::Matrix4f rotScaleM(Eigen::Matrix4f::Identity());
			
			while (!linkN.isNull()) {		
				QDomElement linkDataE = linkN.toElement();
				if (linkDataE.tagName() == "Joint") {										
					
					QDomNode jointN = linkN.firstChild();
					Eigen::Vector3f jRotation;
					double jLength;
					string jName;
					Joint *jConnector;
					
					cout << "[Human Model] Initializing new joint" << endl;					
					while (!jointN.isNull()) {		
						QDomElement jointDataE = jointN.toElement();
						if (jointDataE.tagName() == "Rotation") {
							QString s = jointDataE.text();
							line = s.toStdString();
							char_separator<char> sep(", ");
							tokenizer<char_separator<char>> tokens(line, sep);
							i = 0;
							BOOST_FOREACH(string t, tokens) {
								x = lexical_cast<double>(t);
								if (i<3) jRotation(i) = x;
								++i;
							}
							cout << "[Human Model] Joint rotation: " 
							<< jRotation(0) << " " << jRotation(1) 
							<< " " << jRotation(2) << endl;
						}
						if (jointDataE.tagName() == "Length") {
							line = jointDataE.text().toStdString();	
							jLength = lexical_cast<double>(line);
							cout << "[Human Model] Joint length: " << jLength << endl;																									
						}						
						if (jointDataE.tagName() == "Name") {							
							jName = jointDataE.text().toStdString();
							cout << "[Human Model] Joint name: " << jName << endl;																									
						}
						if (jointDataE.tagName() == "Connector") {							
							line = jointDataE.text().toStdString();
							if (line == "LinkBase") {
								jConnector = new Joint(Eigen::Vector4f::Zero(), Eigen::Vector3f::Zero(), 0, 0, "BaseJoint");
							} else {
								BOOST_FOREACH(Joint* j, jointList) {
									if (j->name == line) {
										jConnector = j;
										break;
									}
								}
							}
							cout << "[Human Model] Joint connector name: " << jConnector->name << endl;
						}
						jointN = jointN.nextSibling();
					}
					Eigen::Vector4f jBase;
					jBase << 1, 0, 0, 1; 
					Joint* j = new Joint(jBase, jRotation, jLength, jConnector, jName);
					lJointList.push_back(j);
					jointList.push_back(j);
				}
				if (linkDataE.tagName() == "Color") {						
					line = linkDataE.text().toStdString();
					if (line == "red") {
						lColor = red;
						cout << "[Human Model] Link color: red" << endl;
					} else if (line == "blue") {
						lColor = blue;
						cout << "[Human Model] Link color: blue" << endl; 						
					} else {
						lColor = green;
						cout << "[Human Model] Link color: green" << endl; 
					}
				}								
				if (linkDataE.tagName() == "Name") {							
					lName = linkDataE.text().toStdString();
					cout << "[Human Model] Link name: " << lName << endl;																									
				}
				if (linkDataE.tagName() == "SensorId") {
					line = linkDataE.text().toStdString();	
					lSensorId = lexical_cast<int>(line);
					cout << "[Human Model] Link sensor index: " << lSensorId << endl;																									
				}
				if (linkDataE.tagName() == "Connector") {
					line = linkDataE.text().toStdString();
					if (line == "ModelBase") {
						lConnector = new Joint(Eigen::Vector4f::Zero(), Eigen::Vector3f::Zero(), 0, 0, "BaseJoint");
					} else {
						BOOST_FOREACH(Joint* j, jointList) {
							if (j->name == line) { 
								lConnector = j;
								break;
							}
						}

					}
					cout << "[Human Model] Link connector joint: " << lConnector->name << endl;
				}
				
				if (linkDataE.tagName() == "ScaleMatrix") {
					Eigen::Matrix3f T;
					string line = linkDataE.text().toStdString();
					char_separator<char> sep(", ");
					tokenizer<char_separator<char>> tokens(line, sep);
					i = 0;
					BOOST_FOREACH(string t, tokens) {
						x = lexical_cast<double>(t);
						if (i<9) T(i/3, i%3) = x;
						++i;
					}
					
					rotScaleM.block<3, 3>(0, 0) = T;
					cout << "[LPMS] Rotation scale matrix: " <<
						rotScaleM << endl;
				}	
				
				if (linkDataE.tagName() == "SensorOrientation") {
					QString s = linkDataE.text();
					line = s.toStdString();
					char_separator<char> sep(", ");
					tokenizer<char_separator<char>> tokens(line, sep);
					i = 0;
					BOOST_FOREACH(string t, tokens) {
						x = lexical_cast<double>(t);
						if (i<3) sensorOrientation(i) = x;
						++i;
					}
					cout << "[Human Model] Sensor orientation: " 
						<< sensorOrientation(0) << " " << sensorOrientation(1) 
						<< " " << sensorOrientation(2) << endl;					
				}				
				linkN = linkN.nextSibling();
			}
			Link* l = new Link(lConnector, 
				lSensorId, 
				lColor, 
				lName, 
				lJointList,
				rotScaleM,
				sensorOrientation);
			linkList.push_back(l);
		}
		humanModelN = humanModelN.nextSibling();
	}	
}

Joint::Joint(Eigen::Vector4f baseV, Eigen::Vector3f rotV, float length,
	Joint* connector, string name) :
	baseV(baseV), 
	rotV(rotV), 
	length(length),
	connector(connector), 
	name(name),
	localSysV(Eigen::Vector4f::Zero()),
	globalSysV(Eigen::Vector4f::Zero()),
	linkRotM(Eigen::Matrix4f::Identity()),
	globalT(Eigen::Matrix4f::Identity()),
	localT(Eigen::Matrix4f::Identity())	
{	
}

Link::Link(Joint *connector, 
	int sensorId, 
	enum COLORS color,
	string name, 
	vector<Joint*> jointList,
	Eigen::Matrix4f rotScaleM,
	Eigen::Vector3f sensorOrientation) : 
	connector(connector), 
	sensorId(sensorId), 
	color(color),
	name(name), 
	jointList(jointList),
	linkRotM(Eigen::Matrix4f::Identity()),
	sensorOrientation(sensorOrientation),
	offsetSet(false),
	planeAngle(Eigen::Vector3f::Zero()),
	active(true),
	timeout(0),
	rotScaleM(rotScaleM),
	rotOffsetM(Eigen::Matrix4f::Identity()),
	rawRotM(Eigen::Matrix4f::Identity())
{
}		

void HumanModel::resetOffset(void)
{
	BOOST_FOREACH(Link* l, linkList) {
		l->rotOffsetM = l->rawRotM.transpose();
	}
}

Eigen::Matrix3f HumanModel::qToM(float* q)
{
	Eigen::Matrix3f rotationM;

	float qw = q[0];
	float qx = q[1];
	float qy = q[2];
	float qz = q[3];	
	
	rotationM << 	qw*qw + qx*qx - qy*qy - qz*qz, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy,
					2*qx*qy + 2*qw*qz, qw*qw - qx*qx + qy*qy - qz*qz, 2*qy*qz - 2*qw*qx,
					2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, qw*qw - qx*qx - qy*qy + qz*qz;
					
	return rotationM;
}

void HumanModel::updateSensorData(ImuData imuData)
{
	const double d2r = 0.01745;
	if ((unsigned int)imuData.openMatId > linkList.size()) return;
	
	Eigen::Matrix3f M3;		
	
	// The follwoing loop writes the latest sensor information to each joint in the link.
	// For each link 
	BOOST_FOREACH(Link* l, linkList) {
	
		// Checks if sensor ID is assigned to link
		if (l->sensorId == imuData.openMatId) {
		
			// Sets the current rotation matrix to identity
			Eigen::Matrix4f rotationM = Eigen::Matrix4f::Identity();
			
			// Converts fixed sensor offset in Euler angles to rotation matrix
			M3 = Eigen::AngleAxisf(l->sensorOrientation(0) * d2r, Eigen::Vector3f::UnitX()) * 
				Eigen::AngleAxisf(l->sensorOrientation(1) * d2r, Eigen::Vector3f::UnitY()) * 
				Eigen::AngleAxisf(l->sensorOrientation(2) * d2r, Eigen::Vector3f::UnitZ());
				
			// Sets the rotation block (upper-left 3x3 matrix) of current rotation matrix = fixed sensor offset * current sensor rotation 	
			rotationM.block<3, 3>(0, 0) = M3 * qToM(imuData.q);
			l->rawRotM = rotationM;

			// Sets link rotation matrix = current sensor rotation matrix * model initialization offset			
			l->linkRotM = l->rawRotM * l->rotOffsetM;
			
			// Indicates that link has recently been updated with data
			l->timeout = 0;
			l->active = true;
			
			// For each joint in link
			BOOST_FOREACH(Joint *j, l->jointList) {
				// Sets the link rotation matrix in joint
				j->linkRotM = l->linkRotM;
			}
		}
	}
	
	// Recalculates joint positions in link
	recalculateJoints();
}	

void HumanModel::recalculateJoints(void)
{	
	// Calls update on forward kinematics object
	fk->update();
}

void HumanModel::checkTimeout(void)
{
	BOOST_FOREACH(Link* l, linkList)
	{	
		++l->timeout;
		if (l->timeout > 20) {
			// Currently, automatic deactivation of links is not used
			// l->active = false;
		}
	}
}

Eigen::Vector3f HumanModel::quat2Euler(Eigen::Vector4f q)
{
	Eigen::Vector4f sq;
	Eigen::Vector3f a;	
	
	double r2d = 57.2958;
	
	sq = q.cwiseProduct(q);
	
	a(0) = (double)atan2l(2.0*(q(2)*q(3)+q(1)*q(0)), (-sq(1)-sq(2)+sq(3)+sq(0)))*r2d;
	a(1) = (double)asinl(-2.0*(q(1)*q(3)-q(2)*q(0)))*r2d;
	a(2) = (double)atan2l(2.0*(q(0)*q(2)+q(3)*q(0)), (sq(1)-sq(2)-sq(3)+sq(0)))*r2d;
		
	return a;
}

Eigen::Matrix4f HumanModel::quat2Matrix(Eigen::Vector4f q)
{
	Eigen::Matrix4f rotationM;
	double qw = q(0);
	double qx = q(1);
	double qy = q(2);
	double qz = q(3);	
	
	rotationM << 	qw*qw + qx*qx - qy*qy - qz*qz, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy, 0,
					2*qx*qy + 2*qw*qz, qw*qw - qx*qx + qy*qy - qz*qz, 2*qy*qz - 2*qw*qx, 0,	
					2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, qw*qw - qx*qx - qy*qy + qz*qz, 0,
					0, 0, 0, 1;
					
	return rotationM;				
}

void HumanModel::initializeSensors(void) 
{
}

ForwardKinematics::ForwardKinematics(
	vector<Link*> linkList, 
	vector<Joint*> jointList) :
	linkList(linkList),
	jointList(jointList)
{
}

void ForwardKinematics::update(void)
{	
	const float d2r = 0.01745f;	
	
	// This loop does all the calculation to get position information from link orientation.
	// For each link
	BOOST_FOREACH(Link* l, linkList) {	

		// For each joint
		BOOST_FOREACH(Joint *j, l->jointList) {		
			Eigen::Matrix3f jointLocalRotM;		
		
			// Creates local transformation matrix 
			j->localT << 	1, 0, 0, j->connector->localSysV(0),
							0, 1, 0, j->connector->localSysV(1),
							0, 0, 1, j->connector->localSysV(2),
							0, 0, 0, 1;
						
			// Gets rotation matrix from Euler angle joint rotation as defined in model
			jointLocalRotM = Eigen::AngleAxisf(j->rotV(0) * d2r, Eigen::Vector3f::UnitX()) * 
				Eigen::AngleAxisf((float) j->rotV(1) * d2r, Eigen::Vector3f::UnitY()) * 
				Eigen::AngleAxisf((float) j->rotV(2) * d2r, Eigen::Vector3f::UnitZ());
			
			// Sets the rotation block of the local rotation / transformation matrix 
			j->localT.block<3, 3>(0, 0) = jointLocalRotM; 
			
			// Scales the base vector of the joint. Vector is always (1, 0, 0, 1).
			Eigen::Vector4f scaledBaseV;
			scaledBaseV << 	j->baseV(0) * j->length,
							j->baseV(1) * j->length,
							j->baseV(2) * j->length,
							j->baseV(3);
			
			// Rotates base vector by local rotation matrix
			j->localSysV = j->localT * scaledBaseV;
			
			// Rotates local system vector by link rotation matrix and adds connector translation
			j->globalSysV = l->linkRotM * j->localSysV + l->connector->globalSysV;
			
			// Puts together global translation vector
			j->globalT = l->linkRotM * j->localT;	
		}
		l->planeAngle = projectionAngle(l->linkRotM.block<3, 3>(0, 0));
	}
}

Eigen::Vector3f ForwardKinematics::projectionAngle(Eigen::Matrix3f rm)
{
	Eigen::Vector3f xv;
	Eigen::Vector3f yv;
	Eigen::Vector3f zv;	
	Eigen::Vector3f pv;
	Eigen::Vector3f v;
	Eigen::Vector3f a;
	
	float r2d = 57.2958f;	
	
	xv << 1, 0, 0;
	yv << 0, 1, 0;
	zv << 0, 0, 1;
	
	v = rm * yv;	
	pv = v - (v.dot(xv) * xv);
	a(0) = acos(pv.dot(zv)) * r2d;

	v = rm * zv;		
	pv = v - (v.dot(yv) * yv);
	a(1) = acos(pv.dot(xv)) * r2d;

	v = rm * yv;		
	pv = v - (v.dot(zv) * zv);
	a(2) = acos(pv.dot(xv)) * r2d;
	
	return a;
}