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

#ifndef HUMAN_MODEL_WINDOW
#define HUMAN_MODEL_WINDOW

#include <QGLWidget>
#include <QWidget>
#include <QTimer>
#include <QMouseEvent>

#include <string>
#include <iostream>
#include <limits>
using namespace std;

#include "ImuData.h"
#include "HumanModel.h"

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
using namespace boost;

#include <Eigen/Dense>
#include <Eigen/Geometry>

class QGLShaderProgram;

// Contains a 3D representation of the human model
class HumanModelWindow : public QGLWidget
{
    Q_OBJECT

public:
	// Constructor
    HumanModelWindow(HumanModel *hm, QWidget *parent = 0, QGLWidget *shareWidget = 0);
	
	// Destructor
    ~HumanModelWindow(void);
	
	// Returns minimum size of windo
    QSize minimumSizeHint() const;
	
	// Returns size hint
    QSize sizeHint() const;
	
	// Draws coordinate system
	void drawAxes(void);

	// Draws a triangle
	void drawTri(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, bool fill);
	
	// Draws the current human model data
	void drawHumanModel(void);
	
	// Draws a Vicon Blade inspired diamond to represent a joing in the model
	void drawDiamond(Eigen::Matrix3f R, Eigen::Vector3f t, double l, enum COLORS c, bool active);
	
	// Draws the floor of the scene
	void drawFloor(void);
	
	// Initializes OpenGL
    void initializeGL();
	
	// Redraws scene
    void paintGL();
	
	// Resizes scene
    void resizeGL(int width, int height);
	
	// Draws a link
	void drawLink(Link* l);	
	
	// Rotates scene
	void rotateBy(int xAngle, int yAngle, int zAngle);
	
	// Rotates scene by a relative angle
	void rotateSceneBy(int xAngle, int yAngle, int zAngle);
	
	// Draws scene background
	void drawBackground(void);
	
public slots:	
	// Updates current rotations quaternion data
	void updateQuaternion(ImuData imuData);
	
	// Updates whole window
	void updateWindow(void);	
	
	// Is called in case of a mouse button pressed event
	void mousePressEvent(QMouseEvent *event);
	
	// Is called in case mouse is moved
	void mouseMoveEvent(QMouseEvent *event);
	
public:
	HumanModel *hm;
	ImuData imuData;	
	QPoint lastPos;
	int xRot;
	int yRot;
	int zRot;
	int xSRot;
	int ySRot;
	int zSRot;
};

#endif
