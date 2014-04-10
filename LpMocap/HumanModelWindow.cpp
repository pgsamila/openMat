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

#include "HumanModelWindow.h"

#include <iostream>
using namespace std;

std::mutex render_mutex;

HumanModelWindow::HumanModelWindow(HumanModel *hm, 
	QWidget *parent, 
	QGLWidget *shareWidget) : 
	QGLWidget(parent, shareWidget),
	lastPos(0, 0),
	xRot(0),
	yRot(0),
	zRot(0),
	xSRot(0),
	ySRot(0),
	zSRot(0),
	hm(hm)
{	
	glob_translate_x = 0.0f;
	glob_translate_y = 0.0f;
	glob_translate_z = 0.0f;
	
	video_writer = new cv::VideoWriter();
	video_recording_started_ = false;
}

void HumanModelWindow::updateWindow(void)
{
	updateGL();
}

HumanModelWindow::~HumanModelWindow(void)
{
	delete video_writer;
}

QSize HumanModelWindow::minimumSizeHint() const
{
    return QSize(960, 480);
}

QSize HumanModelWindow::sizeHint() const
{
    return QSize(960, 480);
}

void HumanModelWindow::initializeGL()
{
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);	
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	static GLfloat lightPosition[4] = { 0.0, 0.0, 7.0, 1.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glEnable(GL_COLOR_MATERIAL);
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	
	glLineWidth(1.0f);
}

bool HumanModelWindow::OpenVideoFile(const char *fn)
{
	cv::Size image_size(this->width(), this->height());
	if (video_writer->open(fn, CV_FOURCC('M', 'J', 'P', 'G'), 30, image_size) == false) {
		printf("[HumanModelWindow] Couldn't open video file %s\n", fn);	
		return false;
	}
	
	video_recording_started_ = true;
	
	return true;
}

cv::Mat QImage2Mat(QImage const& src)
{
	src.convertToFormat(QImage::Format_RGB888);
	cv::Mat tmp(src.height(), src.width(), CV_8UC3, (uchar*)src.bits(), src.bytesPerLine());
	cv::Mat result;
	cv::cvtColor(tmp, result, CV_BGR2RGB);
	
	return result;
}

void HumanModelWindow::WriteVideoFrame(void)
{
	cv::Mat reversed_image;
	cv::Mat flipped_image;

	if (video_writer->isOpened() == false) return;
		
	int width = this->width();
	int height = this->height();
	
	unsigned char* buffer = new unsigned char[width*height*3];
	
	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buffer);
	cv::Mat opengl_image(height, width, CV_8UC3, buffer);
	cv::cvtColor(opengl_image, reversed_image, CV_BGR2RGB);
	cv::flip(reversed_image, flipped_image, 0);
	video_writer->write(flipped_image);
	
	delete buffer;
}

void HumanModelWindow::CloseVideoFile(void)
{
	if (video_writer->isOpened() == false) return;
	
	video_recording_started_ = false;
	
	delete video_writer;
	
	video_writer = new cv::VideoWriter();	
}	

void HumanModelWindow::updateQuaternion(ImuData imuData)
{
	this->imuData = imuData;
}

void HumanModelWindow::paintGL()
{	
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

	glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

    glTranslatef(glob_translate_x, glob_translate_y, glob_translate_z);

    glTranslatef(0.0, -1.0, -10.0);

	glRotatef(15, 1.0f, 0.0f, 0.0f);
	/* glRotatef(-30, 0.0f, 1.0f, 0.0f); */
	
	// drawBackground();	
	drawFloor();
	// drawAxes();	
	drawHumanModel();
}

void HumanModelWindow::drawHumanModel(void)
{
	Eigen::Vector3f t = Eigen::Vector3f::Zero();
	Eigen::Vector3f r = Eigen::Vector3f::Zero();

	glPushMatrix();
	glTranslatef(t(0), t(1), t(2));
    glRotatef(r(0), 1.0f, 0.0f, 0.0f);
    glRotatef(r(1), 0.0f, 1.0f, 0.0f);
    glRotatef(r(2), 0.0f, 0.0f, 1.0f);

	glColor3f(1.0, 0, 0);

	bool joint_done[64];
	for (int i=0; i < hm->mChannelCount; ++i) joint_done[i] = false;
	
	for (int i=0; i < hm->mChannelCount; ++i) {
		for (int j=0; j < hm->mChannelCount; ++j) {
			if (hm->GetChannelParent(j) == i && joint_done[j] == false) {
				joint_done[j] = true;
				
				Eigen::Matrix3f rotation_matrix;
				Eigen::Vector3f vector_1;
				Eigen::Vector3f vector_2;
				Eigen::Vector3f dir_vector;
				float vector_length;
				
				vector_1 << hm->GetDataTX(i), hm->GetDataTY(i), hm->GetDataTZ(i);
				vector_2 << hm->GetDataTX(j), hm->GetDataTY(j), hm->GetDataTZ(j);

				vector_1 = vector_1 * 0.01f;
				vector_2 = vector_2 * 0.01f;
				dir_vector = vector_2 - vector_1;
				
				rotation_matrix = Eigen::Quaternionf().setFromTwoVectors(Eigen::Vector3f(1.0f, 0, 0), dir_vector);
				
				vector_length = (vector_1 - vector_2).norm();
				
				/* glDisable(GL_LIGHTING);
				glColor3f(1.0f, 0.0f, 0.0f);		
				glLineWidth(3.0f);
				glBegin(GL_LINES);
				glVertex3f(vector_1(0),  vector_1(1), vector_1(2));
				glVertex3f(vector_2(0),  vector_2(1), vector_2(2));
				glEnd();
				glEnable(GL_LIGHTING); */

				drawDiamond(rotation_matrix, vector_1, vector_length, blue, true);
			}
		}
	}

	glPopMatrix();
}

void HumanModelWindow::drawLink(void)
{
	const double d2r = 0.01745;	
}

void HumanModelWindow::drawDiamond(Eigen::Matrix3f R, Eigen::Vector3f t, double l, enum COLORS c, bool active)
{
	int s = 4;	
	double r = l / 7;
	
	for (int i=0; i < s; i++) {
		if (active == true) {
			if (i < s / 8) {	
				switch (c) {
				case blue:
					glColor3f(0.6f, 0.6f, 0.8f);
					break;

				case red:
					glColor3f(0.8f, 0.6f, 0.6f);
					break;

				case green:
					glColor3f(0.6f, 0.8f, 0.6f);
					break;
				
				default:
					glColor3f(0.6f, 0.6f, 0.8f);
				};							
			} else {
				switch (c) {
				case blue:
					glColor3f(0.3f, 0.3f, 0.8f);
					break;

				case red:
					glColor3f(0.8f, 0.3f, 0.3f);
					break;

				case green:
					glColor3f(0.3f, 0.8f, 0.3f);
					break;
				
				default:
					glColor3f(0.3f, 0.3f, 0.8f);
				};	
			}
		} else {
			if (i < s / 8) {		
				switch (c) {
				case blue:
					glColor3f(0.2f, 0.2f, 0.4f);
					break;

				case red:
					glColor3f(0.4f, 0.2f, 0.2f);
					break;

				case green:
					glColor3f(0.2f, 0.4f, 0.2f);
					break;
				
				default:
					glColor3f(0.2f, 0.2f, 0.4f);
				};							
			} else {
				switch (c) {
				case blue:
					glColor3f(0.0f, 0.0f, 0.2f);
					break;

				case red:
					glColor3f(0.2f, 0.0f, 0.0f);
					break;

				case green:
					glColor3f(0.0f, 0.2f, 0.0f);
					break;
				
				default:
					glColor3f(0.0f, 0.0f, 0.2f);
				};	
			}
		}

		Eigen::Vector3f p0, p1, p2, p3;
	
		double a0 = 2 * M_PI / s * (double) i + M_PI * 3 / 8;
		double x0 = cos(a0) * r;
		double y0 = sin(a0) * r;
		
		double a1 = 2 * M_PI / s * (double) (i + 1) + M_PI * 3 / 8;
		double x1 = cos(a1) * r;
		double y1 = sin(a1) * r;		
		
		p0 = Eigen::Vector3f(0, 0, 0);
		p1 = Eigen::Vector3f(l*1/3, x1, y1);
		p2 = Eigen::Vector3f(l*1/3, x0, y0);
		p3 = Eigen::Vector3f(l, 0, 0);
		
		p0 = (R * p0) + t;
		p1 = (R * p1) + t;
		p2 = (R * p2) + t;
		p3 = (R * p3) + t;		
		
		drawTri(p0, p1, p2, true);
		drawTri(p3, p2, p1, true);
		
		glColor4f(0.0f, 0.0f, 0.5f, 1.0f);
		drawTri(p0, p1, p2, false);
		drawTri(p3, p2, p1, false);		
	}
	
	glPopMatrix();		
}

void HumanModelWindow::drawAxes(void)
{
	QFont qf; 
	qf.setPixelSize(30);
	
	// glLineWidth(2.0f);

	glDisable(GL_LIGHTING);
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(30.0, 0.0, 0.0f);
	glVertex3f(-30.0, 0.0, 0.0f);
	glEnd();
	glColor3f(1.0f, 0.0f, 0.0f);
	// renderText(1.2, 0.1, +0.1, QString("X"), qf);
	
	glColor3f(0.0, 1.0, 0.0);	
	glBegin(GL_LINES);
	glVertex3f(0.0, -30.0, 0.0f);
	glVertex3f(0.0, 30.0, 0.0f);
	glEnd();	
	glColor3f(0.0f, 1.0f, 0.0f);
	// renderText(0.1, 1.2, +0.1, QString("Y"), qf);	
	
	glColor3f(0.0, 0.0, 1.0);	
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, -30.0f);
	glVertex3f(0.0, 0.0, 30.0f);
	glEnd();
	glColor3f(0.0f, 0.0f, 1.0f);
	// renderText(0.1, 0.0, -1.2, QString("Z"), qf);	
	glEnable(GL_LIGHTING);	
	
	glLineWidth(1.0f);	
}

void HumanModelWindow::drawFloor(void)
{
	glDisable(GL_LIGHTING);
	glColor3f(0.8f, 0.8f, 0.8f);		

	glBegin(GL_LINES);
	for (float i=-10; i <= 10; i+=0.5) {
		if (i==0) { 
			glColor3f(0.8f, 0.8f, 0.8f); 
		} else { 
			glColor3f(0.8f, 0.8f, 0.8f); 
		};
		glVertex3f(i, -0.01f, -10.0f);
		glVertex3f(i, -0.01f, 10.0f);
		if (i==0) { 
			glColor3f(0.7f, 0.7f, 0.7f); 
		} else { 
			glColor3f(0.6f, 0.6f, 0.6f); 
		};
		glVertex3f(-10.0f, -0.01f, i);
		glVertex3f(10.0f, -0.01f, i);
	};
	glEnd();
	glEnable(GL_LIGHTING);	
}

void HumanModelWindow::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side*2) / 2, (height - side) / 2, side*2, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	// glFrustum(-0.6, +0.6, -0.6, +0.6, 4.0, 100.0);
	glFrustum(-1.2, +1.2, -0.6, +0.6, 4.0, 100.0);
    glMatrixMode(GL_MODELVIEW);	
}

void HumanModelWindow::drawTri(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, bool fill)
{
	double qx, qy, qz, px, py, pz;	

	double f0x = p0(0);
	double f0y = p0(1);
	double f0z = p0(2);
	
	double f1x = p1(0);
	double f1y = p1(1);
	double f1z = p1(2);
	
	double f2x = p2(0);
	double f2y = p2(1);
	double f2z = p2(2);
	
	if (fill == true)
	{
		glBegin(GL_TRIANGLES);			
	} else {
		glBegin(GL_LINE_LOOP);
	}

	Eigen::Vector3f a, b, c, d;
	a = p1 - p0;
	b = p2 - p0;
	c = a.cross(b);
	d = c / c.norm();	
 	glNormal3f(d(0), d(1), d(2));	
	
	glVertex3f(f0x, f0y, f0z);	
	glVertex3f(f1x, f1y, f1z);
	glVertex3f(f2x, f2y, f2z);
	
	qx = f2x - f0x;
	qy = f2y - f0y;
	qz = f2z - f0z;
	px = f1x - f0x;
	py = f1y - f0y;
	pz = f1z - f0z;

	double fnx = py*qz - pz*qy;
	double fny = pz*qx - px*qz;
	double fnz = px*qy - py*qx;
	
	glEnd();	
}

void HumanModelWindow::rotateBy(int xAngle, int yAngle, int zAngle)
{
    xRot += xAngle;
    yRot += yAngle;
    zRot += zAngle;
	
    updateGL();
}

void HumanModelWindow::rotateSceneBy(int xAngle, int yAngle, int zAngle)
{
    xSRot += xAngle;
    ySRot += yAngle;
    zSRot += zAngle;
	
    updateGL();
}

void HumanModelWindow::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void HumanModelWindow::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastPos.x();	
	int dy = event->y() - lastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		rotateBy(8*dy, 8*dx, 0);
	} else if (event->buttons() & Qt::RightButton) {
		glob_translate_x -= (float)dx / 32.0f;
		glob_translate_y += (float)dy / 32.0f;
	}
	lastPos = event->pos();
}

void HumanModelWindow::wheelEvent(QWheelEvent *event)
{
	glob_translate_z += (float)event->angleDelta().y() / 100.0f;
}

void HumanModelWindow::drawBackground(void)
{
	glDisable(GL_CULL_FACE);		
	glDisable(GL_LIGHTING);
	glBegin(GL_QUADS);

	glColor3f((GLfloat) 0.8, (GLfloat) 0.8, (GLfloat) 1.0);
	glVertex3f(-10.0, 10.0, -10.0);
	glVertex3f(-10.0,-10.0, -10.0);

	glColor3f((GLfloat) 0.2, (GLfloat) 0.2, (GLfloat) 1.0);
	glVertex3f(10.0, -10.0, -10.0);
	glVertex3f(10.0, 10.0, -10.0);
	glEnd();	
	glEnable(GL_LIGHTING);	
	glEnable(GL_CULL_FACE);
}

bool HumanModelWindow::IsVideoRecordingStarted(void)
{
	return video_recording_started_;
}