/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
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

#ifndef FIELD_MAP_DISPLAY
#define FIELD_MAP_DISPLAY

#include <QtGui>
#include <QGLWidget>
#include <QComboBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QtOpenGL>

#include <GL/glu.h>

#include <string>
#include <iostream>
using namespace std;

#include "ImuData.h"
#include "ObjFileParser.h"
#include "LpmsDefinitions.h"

#include <Eigen/Dense>	

class FieldMapDisplay : public QGLWidget
{
    Q_OBJECT

public:
	QPoint lastPos;
	int xRot;
	int yRot;
	int zRot;
	int xSRot;
	int ySRot;
	int zSRot;
	Eigen::Vector3f fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW];
	Eigen::Vector3f hardIronOffset;
	Eigen::Matrix3f softIronMatrix;
	Eigen::Vector3f currentField;
	float fieldRadius;
	bool fieldUpdated;
	bool showOriginalField;
	bool showCorrectedField;
	bool showEllipsoid;
	
    FieldMapDisplay(QWidget *parent = 0, QGLWidget *shareWidget = 0);
    ~FieldMapDisplay();
    QSize minimumSizeHint() const;
    QSize sizeHint() const;
	void drawAxes(void);
    void initializeGL();
    void resizeGL(int width, int height);
	void drawTri(Eigen::Vector3f p0, 
		Eigen::Vector3f p1, 
		Eigen::Vector3f p2,
		bool fill);
	void drawQuad(Eigen::Vector3f p0, 
		Eigen::Vector3f p1, 
		Eigen::Vector3f p2, 
		Eigen::Vector3f p3, 
		bool fill);
	void drawBackground(void);	
	void drawCylinder(float l, float r, Eigen::Matrix3f m, Eigen::Vector3f t);
	void drawCone(float l, float r, Eigen::Matrix4f T4, std::string tipText, Eigen::Vector3f t2);
	void drawSphere(double r, Eigen::Vector3f t);
	void drawGridCube(void);
	void drawFieldMap(void);

signals:
    void clicked();

public slots:	
	void updateWindow(void);
	void rotateBy(int xAngle, int yAngle, int zAngle);
	void rotateSceneBy(int xAngle, int yAngle, int zAngle);
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void paintEvent(QPaintEvent*);
	void updateFieldMap(void);
};

#endif
