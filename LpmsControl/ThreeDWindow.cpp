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

#include <QtGui>
#include <QtOpenGL>

#include "ThreeDWindow.h"

#include <iostream>
using namespace std;

Eigen::Vector3f scale;

ThreeDWindow::ThreeDWindow(QWidget *parent, QGLWidget *shareWidget)
    : QGLWidget(parent, shareWidget),
	xRot(0),
	yRot(0),
	zRot(0),
	xSRot(0),
	ySRot(0),
	zSRot(0)
{
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(updateWindow()));
    timer->start(33);

#ifdef USE_POINT_CUBE
	caseObj.parse("LpmsCase.obj");
#endif
	lpmsCaseScale = 0.085f;	

	scale << lpmsCaseScale, lpmsCaseScale, -lpmsCaseScale;
	for (unsigned int i=0; i<caseObj.faceList.size(); i++) {
		caseObj.faceList[i].vertexList[0] = (caseObj.faceList[i].vertexList[0] - caseObj.centerVertex).cwiseProduct(scale);
		caseObj.faceList[i].vertexList[1] = (caseObj.faceList[i].vertexList[1] - caseObj.centerVertex).cwiseProduct(scale);
		caseObj.faceList[i].vertexList[2] = (caseObj.faceList[i].vertexList[2] - caseObj.centerVertex).cwiseProduct(scale);
	}
	
	zeroImuData(&imuData);
	
	rM = Eigen::Matrix3f::Identity();
}

void ThreeDWindow::zeroImuData(ImuData* id)
{
	id->q[0] = 1.0f;
	id->q[1] = 0.0f;
	id->q[2] = 0.0f;
	id->q[3] = 0.0f;
	
	for (int i=0; i<3; i++) id->r[i] = 0.0f;
	for (int i=0; i<3; i++) id->a[i] = 0.0f;
	for (int i=0; i<3; i++) id->g[i] = 0.0f;
	for (int i=0; i<3; i++) id->b[i] = 0.0f;
	for (int i=0; i<3; i++) id->aRaw[i] = 0.0f;
	for (int i=0; i<3; i++) id->gRaw[i] = 0.0f;
	for (int i=0; i<3; i++) id->bRaw[i] = 0.0f;
	
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			if (i != j) {
				id->rotationM[i*3+j] = 0.0f;
				id->rotOffsetM[i*3+j] = 0.0f;
			} else {
				id->rotationM[i*3+j] = 1.0f;
				id->rotOffsetM[i*3+j] = 1.0f;
			}
		}
	}
	
	id->openMatId = 1;	
	id->frameCount = 0;
	id->timeStamp = 0.0f;
}

void ThreeDWindow::updateWindow(void)
{
	updateGL();
}

ThreeDWindow::~ThreeDWindow()
{
}

QSize ThreeDWindow::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize ThreeDWindow::sizeHint() const
{
    return QSize(100, 100);
}

void ThreeDWindow::initializeGL()
{
	static GLfloat lightPosition[4] = { 0.0, 0.0, 10.0, 0.75 };

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);	
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glEnable(GL_COLOR_MATERIAL);
	
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
}

const float d2r = 0.01745f;	

void ThreeDWindow::updateQuaternion(ImuData imuData)
{
	this->imuData = imuData;
	
	Eigen::Matrix3f M;
	Eigen::Matrix3f scale;
	Eigen::Matrix3f tM;

	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			tM(i, j) = imuData.rotationM[i*3+j];
		}
	}
	
	scale <<	1, 0, 0,
				0, 0, -1,
				0, 1, 0;	
				
	M = Eigen::AngleAxisf(-90 * d2r, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(0 * d2r, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(0 * d2r, Eigen::Vector3f::UnitZ());
		
	rM = M * tM.transpose() * scale;
}

void ThreeDWindow::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -20.0f);

	drawBackground();

    glRotatef(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
	glRotatef(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
	glRotatef(zRot / 16.0f, 0.0f, 0.0f, 1.0f);

	glRotatef(15, 1.0f, 0.0f, 0.0f);
	glRotatef(30, 0.0f, 1.0f, 0.0f);
		
	drawAxes();
		
	Eigen::Matrix3f M;
	Eigen::Matrix4f rM4;
	Eigen::Matrix4f T4;
	Eigen::Vector3f l;
	float radius = 0.1f;

#define USE_POINT_CUBE
#ifdef USE_POINT_CUBE
	drawPointCube();
	l = Eigen::Vector3f(2.4f, 0.9f, 1.4f);
#endif

#ifdef USE_LPMS_CUBE
	drawLpmsCase();
	l = (caseObj.maxVertex - caseObj.centerVertex) * lpmsCaseScale + Eigen::Vector3f(0.35f, 0.35f, 0.35f);
#endif
	
	glColor3f((GLfloat) 0.0f, (GLfloat) 0.0f, (GLfloat) 0.8f);		
	M = Eigen::AngleAxisf(0 * d2r, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(90 * d2r, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	
	T4 <<	M(0, 0), M(0, 1), M(0, 2), l(0),
			M(1, 0), M(1, 1), M(1, 2), 0,
			M(2, 0), M(2, 1), M(2, 2), 0,
			0, 0, 0, 1;			
	
	rM4 <<	rM(0, 0), rM(0, 1), rM(0, 2), 0,
			rM(1, 0), rM(1, 1), rM(1, 2), 0,
			rM(2, 0), rM(2, 1), rM(2, 2), 0,
			0, 0, 0, 1;
		
	drawCylinder(l(0), radius / 2.0f, rM * M);
	drawCone(0.3f, radius, rM4 * T4, ""); // "X");
	
	glColor3f((GLfloat) 0.8f, (GLfloat) 0.0f, (GLfloat) 0.0f);		
	M = Eigen::AngleAxisf(90.0f * d2r, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(180.0f * d2r, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

	T4 <<	M(0, 0), M(0, 1), M(0, 2), 0,
			M(1, 0), M(1, 1), M(1, 2), l(1),
			M(2, 0), M(2, 1), M(2, 2), 0,
			0, 0, 0, 1;			
	
	rM4 <<	rM(0, 0), rM(0, 1), rM(0, 2), 0,
			rM(1, 0), rM(1, 1), rM(1, 2), 0,
			rM(2, 0), rM(2, 1), rM(2, 2), 0,
			0, 0, 0, 1;
			
	drawCylinder(l(1), radius / 2, rM * M);
	drawCone(0.3f, radius, rM4 * T4, ""); // "Z");
	
	glColor3f((GLfloat) 0.0, (GLfloat) 0.8, (GLfloat) 0.0);		
	M = Eigen::AngleAxisf(0 * d2r, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(180 * d2r, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	
	T4 <<	M(0, 0), M(0, 1), M(0, 2), 0,
			M(1, 0), M(1, 1), M(1, 2), 0,
			M(2, 0), M(2, 1), M(2, 2), -l(2),
			0, 0, 0, 1;			
	
	rM4 <<	rM(0, 0), rM(0, 1), rM(0, 2), 0,
			rM(1, 0), rM(1, 1), rM(1, 2), 0,
			rM(2, 0), rM(2, 1), rM(2, 2), 0,
			0, 0, 0, 1;	
		
	drawCylinder(l(2), radius / 2, rM * M);
	drawCone(0.3f, radius, rM4 * T4, ""); // "Y");
}

void ThreeDWindow::drawBackground(void)
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

void ThreeDWindow::updateFieldMap(float magField[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW][3],
	float hardIronOffset[3], float softIronMatrix[3][3]) 
{
	for (int i=0; i<ABSMAXPITCH; i++) {
		for (int j=0; j<ABSMAXROLL; j++) {
			for (int k=0; k<ABSMAXYAW; k++) {
				for (int l=0; l<3; l++) {
					this->fieldMap[i][j][k](l) = magField[i][j][k][l];
				}
			}
		}
	}
	
	for (int i=0; i<3; i++) {
		this->hardIronOffset(i) = hardIronOffset[i];
	}
	
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			this->softIronMatrix(i, j) = softIronMatrix[i][j];
		}
	}
}

void ThreeDWindow::drawFieldMap(void)
{
	float ps = 90.0f / ABSMAXPITCH;
	float rs = 180.0f / ABSMAXROLL;
	float ys = 180.0f / ABSMAXYAW;
	
	float p = 0.0f;
	float r = 0.0f;
	float y = 0.0f;
	float l = 0.0f;

	Eigen::Vector3f v;
	Eigen::Vector3f v2;
	Eigen::Matrix3f M;
	
	float s = (1.0f / 50.0f);
	
	for (int i=0; i<ABSMAXPITCH; i++) {
		for (int j=0; j<ABSMAXROLL; j++) {
			for (int k=0; k<ABSMAXYAW; k++) {							
				v = fieldMap[i][j][k] * s;
				v2 = fieldMap[i][j][k] - hardIronOffset;
				v2 = (softIronMatrix * v2) * s;

				if (v.norm() > 0) {
					glColor3f(0.0, 1.0, 0.0);
					drawSphere(v, 0.05f);
			
					glColor3f(0.0, 0.0, 1.0);
					drawSphere(v2, 0.05f);
				}

				y += ys;
			}
			r += rs;
		}
		p += ps;
	}
}

void ThreeDWindow::drawSphere(Eigen::Vector3f p, float r)
{
	int s = 5;

	for (int j=0; j < s; j++) {			
		for (int i=0; i < s; i++) {
			float aj0 = M_PI / (float) s * (float) j;
			float z0 = cos(aj0) * r;
			float r0 = sin(aj0) * r;
						
			float ai0 = 2 * M_PI / (float) s * (float) i;
			float x0 = cos(ai0) * r0;
			float y0 = sin(ai0) * r0;
		
			float ai1 = 2 * M_PI / (float) s * (float) (i + 1);
			float x1 = cos(ai1) * r0;
			float y1 = sin(ai1) * r0;

			float aj1 = M_PI / (float) s * (float) (j + 1);
			float z1 = cos(aj1) * r;
			float r1 = sin(aj1) * r;
						
			float x3 = cos(ai0) * r1;
			float y3 = sin(ai0) * r1;
		
			float x2 = cos(ai1) * r1;
			float y2 = sin(ai1) * r1;

			drawQuad(Eigen::Vector3f(x0, y0, z0)+p,
				Eigen::Vector3f(x1, y1, z0)+p,
				Eigen::Vector3f(x2, y2, z1)+p,
				Eigen::Vector3f(x3, y3, z1)+p,
				true);
		}
	}
}

void ThreeDWindow::drawGridCube(void)
{
	QFont f;
	f.setPixelSize(30);	

	glDisable(GL_CULL_FACE);		
	glDisable(GL_LIGHTING);
	
	glColor3f((GLfloat) 0.9f, (GLfloat) 0.9f, (GLfloat) 0.9f);		

	glLineWidth(1.0f);		
	
	for (int i=0; i<3; i++) {
		for (float y=-2.0; y<=2; y+=2.0) {
			for (float z=-2.0; z<=2; z+=2.0) {
				switch(i) {
				case 0:
					glBegin(GL_LINES);
						glVertex3f(-2.0f, y, z);	
						glVertex3f(2.0f, y, z);			
					glEnd();
				break;

				case 1:
					glBegin(GL_LINES);
						glVertex3f(y, -2.0f, z);	
						glVertex3f(y, 2.0f, z);			
					glEnd();
				break;

				case 2:
					glBegin(GL_LINES);
						glVertex3f(y, z, -2.0f);	
						glVertex3f(y, z, 2.0f);			
					glEnd();
				break;
				}	
			}
		}
	}
	
	glColor3f((GLfloat) 0.0f, (GLfloat) 1.0f, (GLfloat) 0.0f);		
	renderText(0 + 0.05, 0 + 0.05, 2.0 + 0.05, QString("Z"), f);	
	glColor3f((GLfloat) 0.0f, (GLfloat) 0.0f, (GLfloat) 1.0f);	
	renderText(0 + 0.05, 2.0 + 0.05, 0.0 + 0.05, QString("Y"), f);		
	glColor3f((GLfloat) 1.0f, (GLfloat) 0.0f, (GLfloat) 0.0f);	
	renderText(2.0 + 0.05, 0 + 0.05, 0.0 + 0.05, QString("X"), f);		
		
	glLineWidth(1.0f);			
		
	glEnable(GL_CULL_FACE);		
	glEnable(GL_LIGHTING);
}

void ThreeDWindow::drawLpmsCase(void)
{
	std::vector<ObjFace> faceList = caseObj.getFaceList();

	glColor3f((GLfloat) 0.9, (GLfloat) 0.9, (GLfloat) 0.9);		
	
	for (unsigned int i=0; i<faceList.size(); i++) {
		drawTri(rM * faceList[i].vertexList[2],
			rM * faceList[i].vertexList[1],
			rM * faceList[i].vertexList[0], true);
		drawTri(rM * faceList[i].vertexList[0],
			rM * faceList[i].vertexList[1],
			rM * faceList[i].vertexList[2], true);
	}
}
	
void ThreeDWindow::drawPointCube(void)
{
	float l = 2.0;
	Eigen::Vector3f p1, p2, p3, p4, p5, p6, p7, p8;

	p1 << l, l/4, -l/2;
	p2 << l, -l/4, -l/2;
	p3 << -l, -l/4, -l/2;
	p4 << -l, l/4, -l/2;

	p5 << l, l/4, l/2;
	p6 << l, -l/4, l/2;
	p7 << -l, -l/4, l/2;
	p8 << -l, l/4, l/2;	

	p4 = rM * p4;
	p1 = rM * p1;
	p2 = rM * p2;
	p3 = rM * p3;
	p5 = rM * p5;
	p6 = rM * p6;
	p7 = rM * p7;
	p8 = rM * p8;
	
	glColor3f((GLfloat) 1.0, (GLfloat) 1.0, (GLfloat) 1.0);		
	
	drawQuad(p1, p2, p3, p4, true);	
	drawQuad(p8, p7, p6, p5, true);			
	drawQuad(p6, p7, p3, p2, true);	
	drawQuad(p3, p7, p8, p4, true);		
	drawQuad(p8, p5, p1, p4, true);		
	drawQuad(p5, p6, p2, p1, true);
	
	QFont f;
	f.setPixelSize(30);
}

void ThreeDWindow::drawCone(float l, float r, Eigen::Matrix4f T4, std::string tipText)
{
	int s = 10;
			
	Eigen::Vector3f t;
	t << 	T4(0, 3), T4(1, 3), T4(2, 3);

	Eigen::Matrix3f m;
	m <<	T4(0, 0), T4(0, 1), T4(0, 2),
			T4(1, 0), T4(1, 1), T4(1, 2),
			T4(2, 0), T4(2, 1), T4(2, 2);			
				
	for (int i=0; i < s; i++) {
		float a0 = 2 * M_PI / s * (float) i;
		float x0 = cos(a0) * r;
		float y0 = sin(a0) * r;
		
		float a1 = 2 * M_PI / s * (float) (i + 1);
		float x1 = cos(a1) * r;
		float y1 = sin(a1) * r;
		
		drawTri(m * Eigen::Vector3f(x0, y0, 0) + t,
			m * Eigen::Vector3f(x1, y1, 0) + t,
			m * Eigen::Vector3f(0, 0, l) + t, true);
			
		drawTri(m * Eigen::Vector3f(0, 0, l) + t,
			m * Eigen::Vector3f(x1, y1, 0) + t,
			m * Eigen::Vector3f(x0, y0, 0) + t, true);			
	}
	
	QFont f;
	f.setPixelSize(30);	
	Eigen::Vector3f v = m * Eigen::Vector3f(0, 0, l) + t;	
	renderText(v(0) + 0.05, v(1) + 0.05, v(2) + 0.05, QString(tipText.c_str()), f);	
}

void ThreeDWindow::drawCylinder(float l, float r, Eigen::Matrix3f m)
{
	int s = 30;	
	float a = 2.0f * (float) M_PI;
		
	for (int i=0; i < s + 1; i++) {
		float a0 = a / s * (float) i;
		float x0 = cos(a0) * r;
		float y0 = sin(a0) * r;
		
		float a1 = a / s * (float) (i + 1);
		float x1 = cos(a1) * r;
		float y1 = sin(a1) * r;
		
		drawQuad(m * Eigen::Vector3f(x0, y0, 0),
			m * Eigen::Vector3f(x1, y1, 0),
			m * Eigen::Vector3f(x1, y1, l),
			m * Eigen::Vector3f(x0, y0, l), true);
			
		drawQuad(m * Eigen::Vector3f(x0, y0, l),
			m * Eigen::Vector3f(x1, y1, l),
			m * Eigen::Vector3f(x1, y1, 0),
			m * Eigen::Vector3f(x0, y0, 0), true);	
	}
}

void ThreeDWindow::drawAxes(void)
{
	glDisable(GL_LIGHTING);
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(10.0, 0.0, 0.0f);
	glVertex3f(-10.0, 0.0, 0.0f);
	glEnd();

	glColor3f(1.0, 0.0, 0.0);	
	glBegin(GL_LINES);
	glNormal3f(0.0f, 0.0f, 1.0f);	
	glVertex3f(0.0, -10.0, 0.0f);
	glVertex3f(0.0, 10.0, 0.0f);
	glEnd();	
	
	glColor3f(0.0, 1.0, 0.0);	
	glBegin(GL_LINES);
	glNormal3f(0.0f, 1.0f, 0.0f);	
	glVertex3f(0.0, 0.0, -10.0f);
	glVertex3f(0.0, 0.0, 10.0f);
	glEnd();
	glEnable(GL_LIGHTING);	
}

void ThreeDWindow::drawFloor(void)
{
	glBegin(GL_LINES);
	glColor3f(0.8f, 0.8f, 0.8f);	
	for (float i=-10; i <= 10; i+=0.2f) {
		if (i != 0) { 
			glVertex3f(i, 0.0f, -10.0f);
			glVertex3f(i, 0.0f, 10.0f);
			glVertex3f(-10.0f, 0.0f, i);
			glVertex3f(10.0f, 0.0f, i);
		}
	};
	glEnd();
}

void ThreeDWindow::resizeGL(int width, int height)
{
    int side = qMin(width, height);

    glViewport((width - side) / 2, (height - side) / 2, side, side);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	glFrustum(-0.6, +0.6, -0.6, +0.6, 4.0, 100.0);
	glMatrixMode(GL_MODELVIEW);	
	glEnable(GL_NORMALIZE);
}

int ThreeDWindow::heightForWidth( int w ) 
{ 
	return w; 
}

bool ThreeDWindow::hasHeightForWidth()
{
	return true;
}

void ThreeDWindow::drawTri(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, bool fill)
{	
	Eigen::Vector3f a, b, c, d;
	a = p1 - p0;
	b = p2 - p0;
	c = a.cross(b);
	d = c / c.norm();

	if (fill == true) {
		glBegin(GL_TRIANGLES);			
	} else {
		glBegin(GL_LINE_LOOP);
	}
	
	glNormal3f(d(0), d(1), d(2));
	
	glVertex3f(p0(0), p0(1), p0(2));	
	glVertex3f(p1(0), p1(1), p1(2));
	glVertex3f(p2(0), p2(1), p2(2));
	
	glEnd();
}

void ThreeDWindow::drawQuad(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, bool fill)
{
	if (fill == true) {
		drawTri(p0, p1, p2, true);
		drawTri(p3, p0, p2, true);
	} else {
		glBegin(GL_LINE_LOOP);
	
		glVertex3f(p0(0), p0(1), p0(2));	
		glVertex3f(p1(0), p1(1), p1(2));
		glVertex3f(p2(0), p2(1), p2(2));
		glVertex3f(p3(0), p3(1), p3(2));
	
		glEnd();	
	}
}

void ThreeDWindow::setActiveLpms(int openMatId)
{
	activeOpenMatId = openMatId;
}

void ThreeDWindow::rotateBy(int xAngle, int yAngle, int zAngle)
{
    xRot += xAngle;
    yRot += yAngle;
    zRot += zAngle;
    updateGL();
}

void ThreeDWindow::rotateSceneBy(int xAngle, int yAngle, int zAngle)
{
    xSRot += xAngle;
    ySRot += yAngle;
    zSRot += zAngle;
    updateGL();
}

void ThreeDWindow::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void ThreeDWindow::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastPos.x();	
	int dy = event->y() - lastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		rotateBy(8 * dy, 8 * dx, 0);
	} else if (event->buttons() & Qt::RightButton) {
		rotateSceneBy(8 * dy, 8 * dy, 8 * dx);
	}
	
	lastPos = event->pos();
}