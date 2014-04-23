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
#include <QImage>

#include <string>
#include <iostream>
#include <limits>
using namespace std;

#include "ImuData.h"
#include "HumanModel.h"
#include "LinkJoint.h"

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
using namespace boost;

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mutex>

#include "opencv2/opencv.hpp"

class QGLShaderProgram;

class HumanModelWindow : public QGLWidget
{
    Q_OBJECT

public:
    HumanModelWindow(HumanModel *hm, QWidget *parent = 0, QGLWidget *shareWidget = 0);
    ~HumanModelWindow(void);
    QSize minimumSizeHint() const;
    QSize sizeHint() const;
	void drawAxes(void);
	void drawTri(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, bool fill);
	void drawHumanModel(void);
	void drawDiamond(Eigen::Matrix3f R, Eigen::Vector3f t, double l, enum COLORS c, bool active);
	void drawFloor(void);
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
	void drawLink(void);	
	void rotateBy(int xAngle, int yAngle, int zAngle);
	void rotateSceneBy(int xAngle, int yAngle, int zAngle);
	void drawBackground(void);
	bool OpenVideoFile(const char *fn);
	void WriteVideoFrame(void);
	void CloseVideoFile(void);
	bool IsVideoRecordingStarted(void);
	bool checkIfUpperBody(int i);
	
public slots:	
	void updateQuaternion(ImuData imuData);
	void updateWindow(void);	
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	
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
	float glob_translate_x;
	float glob_translate_y;
	float glob_translate_z;
	cv::VideoWriter *video_writer;
	QImage current_frame_buffer_;
	bool video_recording_started_;
	int viewPointIndex;
	bool isShowUpperBody;
};

#endif
