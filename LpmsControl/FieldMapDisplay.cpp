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

#include "FieldMapDisplay.h"

const float d2r = 0.01745f;	

FieldMapDisplay::FieldMapDisplay(QWidget *parent, QGLWidget *shareWidget)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), shareWidget)
{
    xSRot = 0;
    ySRot = 0;
    zSRot = 0;

    xRot = 0;
    yRot = 0;
    zRot = 0;	
	
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(updateWindow()));
    timer->start(33);
	
	showOriginalField = true;
	showCorrectedField = false;
	showEllipsoid = true;
	
	fieldUpdated = false;
	
	for (int i=0; i<ABSMAXPITCH; i++) {
		for (int j=0; j<ABSMAXROLL; j++) {
			for (int k=0; k<ABSMAXYAW; k++) {
				for (int l=0; l<3; l++) {
					this->fieldMap[i][j][k](l) = 0.0f;
				}
			}
		}
	}
	
	for (int i=0; i<3; i++) {
		this->hardIronOffset(i) = 0.0f;
	}
	
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			this->softIronMatrix(i, j) = 0.0f;
		}
	}	
}

void FieldMapDisplay::updateWindow(void)
{
}

FieldMapDisplay::~FieldMapDisplay()
{
}

QSize FieldMapDisplay::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize FieldMapDisplay::sizeHint() const
{
    return QSize(200, 200);
}

void FieldMapDisplay::initializeGL()
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
	
void FieldMapDisplay::paintEvent(QPaintEvent*)
{
    makeCurrent();

	resizeGL(width(), height());
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glShadeModel(GL_SMOOTH);	
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -20.0f);
	
    glRotatef(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
	glRotatef(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
	glRotatef(zRot / 16.0f, 0.0f, 0.0f, 1.0f);

	glRotatef(15, 1.0f, 0.0f, 0.0f);
	glRotatef(30, 0.0f, 1.0f, 0.0f);
	
	drawGridCube();
	drawAxes();
	drawFieldMap();

	int side = qMin(width(), height());

	glDisable(GL_CULL_FACE);
	
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	
	QPen whitePen(Qt::white);	
	QPen redPen(Qt::red);	
	QPen bluePen(Qt::blue);
	QPen greenPen(Qt::green);
	QPen darkGreyPen(Qt::darkGray);
	
	QFont f;
	f.setPixelSize(15);
	painter.setFont(f);
	
	glEnable(GL_CULL_FACE);
	
	painter.end();
}

void FieldMapDisplay::updateFieldMap(void) 
{	
	fieldUpdated = true;
	update();
}

void FieldMapDisplay::drawFieldMap(void)
{
	GLUquadric* quad = gluNewQuadric();

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
	
	float s = 0.03f;
	
	bool showCurrentField = true;
	bool showCurrentFieldLimit = true;
	
	glEnable(GL_BLEND);
	
	for (int i=0; i<ABSMAXPITCH; i++) {
		for (int j=0; j<ABSMAXROLL; j++) {
			for (int k=0; k<ABSMAXYAW; k++) {							
				v = fieldMap[i][j][k] * s;
				v2 = fieldMap[i][j][k] - hardIronOffset;
				v2 = (softIronMatrix * v2) * s;

				if (v.norm() > 0.0f) {
					if (showOriginalField == true && fieldUpdated == true) {
						glPushMatrix();
						glTranslatef(v(0), v(1), v(2));
						glColor3f(1.0, 0.0, 0.0);
						gluSphere(quad, 0.05f, 5, 5);
						glPopMatrix();
					}

					if (showCorrectedField == true && fieldUpdated == true) {
						glPushMatrix();					
						glColor3f(0.0, 0.0, 1.0);
						glTranslatef(v2(0), v2(1), v2(2));
						gluSphere(quad, 0.05f, 5, 5);					
						glPopMatrix();
					}
				}

				y += ys;
			}
			r += rs;
		}
		p += ps;
	}
	
	if (showEllipsoid == true && fieldUpdated == true) {
		glPushMatrix();
		glTranslatef(hardIronOffset(0) * s, hardIronOffset(1) * s, hardIronOffset(2) * s);
		glScalef(fieldRadius / softIronMatrix(0, 0) * s, fieldRadius / softIronMatrix(1, 1) * s, fieldRadius / softIronMatrix(2, 2) * s);
		glColor4f(0.0f, 1.0f, 0.0f, 0.3f);
		gluSphere(quad, 1.0f, 20, 20);
		glPopMatrix();
	}
	
	glDisable(GL_BLEND);
	gluDeleteQuadric(quad);
}

void FieldMapDisplay::drawBackground(void)
{
	glDisable(GL_CULL_FACE);		
	glDisable(GL_LIGHTING);
	
	glBegin(GL_QUADS);

	glColor3f((GLfloat) 0.5, (GLfloat) 0.5, (GLfloat) 0.7);
	glVertex3f(-10.0, 10.0, -10.0);
	glVertex3f(-10.0,-10.0, -10.0);

	glColor3f((GLfloat) 0.0, (GLfloat) 0.0, (GLfloat) 0.7);
	glVertex3f(10.0, -10.0, -10.0);
	glVertex3f(10.0, 10.0, -10.0);
	
	glEnd();	
	
	glEnable(GL_LIGHTING);	
	glEnable(GL_CULL_FACE);
}

void FieldMapDisplay::drawCone(float l, float r, Eigen::Matrix4f T4, std::string tipText, Eigen::Vector3f t2)
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
		
		drawTri(m * Eigen::Vector3f(x0, y0, 0) + t + t2,
			m * Eigen::Vector3f(x1, y1, 0) + t + t2,
			m * Eigen::Vector3f(0, 0, l) + t + t2, true);
			
		drawTri(m * Eigen::Vector3f(0, 0, l) + t + t2,
			m * Eigen::Vector3f(x1, y1, 0) + t + t2,
			m * Eigen::Vector3f(x0, y0, 0) + t + t2, true);			
	}
		
	QFont f;
	f.setPixelSize(30);	
	Eigen::Vector3f v = m * Eigen::Vector3f(0, 0, l) + t + t2;
}

void FieldMapDisplay::drawCylinder(float l, float r, Eigen::Matrix3f m, Eigen::Vector3f t)
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
		
		drawQuad(m * Eigen::Vector3f(x0, y0, 0) + t,
			m * Eigen::Vector3f(x1, y1, 0) + t,
			m * Eigen::Vector3f(x1, y1, l) + t,
			m * Eigen::Vector3f(x0, y0, l) + t, true);
			
		drawQuad(m * Eigen::Vector3f(x0, y0, l) + t,
			m * Eigen::Vector3f(x1, y1, l) + t,
			m * Eigen::Vector3f(x1, y1, 0) + t,
			m * Eigen::Vector3f(x0, y0, 0) + t, true);	
	}
}

void FieldMapDisplay::drawSphere(double r, Eigen::Vector3f t)		
{
	int s = 10;
	
	Eigen::Vector3f v0, v1, v2, v3;
		
	for (int j=0; j < s; j++) {			
		for (int i=0; i < s; i++) {
			double aj0 = M_PI / s * (double) j;
			double z0 = cos(aj0) * r;
			double r0 = sin(aj0) * r;
						
			double ai0 = 2 * M_PI / s * (double) i;
			double x0 = cos(ai0) * r0;
			double y0 = sin(ai0) * r0;
		
			double ai1 = 2 * M_PI / s * (double) (i + 1);
			double x1 = cos(ai1) * r0;
			double y1 = sin(ai1) * r0;

			double aj1 = M_PI / s * (double) (j + 1);
			double z1 = cos(aj1) * r;
			double r1 = sin(aj1) * r;
						
			double x3 = cos(ai0) * r1;
			double y3 = sin(ai0) * r1;
		
			double x2 = cos(ai1) * r1;
			double y2 = sin(ai1) * r1;					

			if (j==0)
			{
				v0 = Eigen::Vector3f(x1, y1, z0) + t;
				v1 = Eigen::Vector3f(x2, y2, z1) + t;
				v2 = Eigen::Vector3f(x3, y3, z1) + t;
			
				drawTri(v2, v1, v0, true);	
			} else {										
				v0 = Eigen::Vector3f(x0, y0, z0) + t;
				v1 = Eigen::Vector3f(x1, y1, z0) + t;
				v2 = Eigen::Vector3f(x2, y2, z1) + t;
				v3 = Eigen::Vector3f(x3, y3, z1) + t;

				drawQuad(v3, v2, v1, v0, true);
			}
		}
	}
}

void FieldMapDisplay::drawGridCube(void)
{
	QFont f;
	f.setPixelSize(18);	

	glDisable(GL_CULL_FACE);		
	glDisable(GL_LIGHTING);
	
	glColor3f((GLfloat) 0.75f, (GLfloat) 0.75f, (GLfloat) 0.75f);		

	glLineWidth(1.0f);		
	
	for (int i=0; i<3; i++) {
		for (float y=-2.0; y<=2; y+=2.0) {
			for (float z=-2.0; z<=2; z+=2.0) {
				if (z != 0 || y != 0) {
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
	}
	
	glColor3f((GLfloat) 0.0f, (GLfloat) 1.0f, (GLfloat) 0.0f);		
	glColor3f((GLfloat) 0.0f, (GLfloat) 0.0f, (GLfloat) 1.0f);		
	glColor3f((GLfloat) 1.0f, (GLfloat) 0.0f, (GLfloat) 0.0f);		
		
	glLineWidth(1.0f);			
		
	glEnable(GL_CULL_FACE);		
	glEnable(GL_LIGHTING);
}
	
void FieldMapDisplay::drawAxes(void)
{
	glLineWidth(1.0f);	

	glDisable(GL_LIGHTING);
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(10.0, 0.0, 0.0f);
	glVertex3f(-10.0, 0.0, 0.0f);
	glEnd();

	glColor3f(0.0, 0.0, 1.0);	
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
	
	glLineWidth(1.0f);		
}

void FieldMapDisplay::resizeGL(int width, int height)
{
    int side = qMin(width, height);

    glViewport((width - side) / 2, (height - side) / 2, side, side);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	glFrustum(-0.6, +0.6, -0.6, +0.6, 4.0, 100.0);
    glMatrixMode(GL_MODELVIEW);	
	glEnable(GL_NORMALIZE);	
}

void FieldMapDisplay::drawTri(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, bool fill)
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

void FieldMapDisplay::drawQuad(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, bool fill)
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

void FieldMapDisplay::rotateBy(int xAngle, int yAngle, int zAngle)
{
    xRot += xAngle;
    yRot += yAngle;
    zRot += zAngle;
}

void FieldMapDisplay::rotateSceneBy(int xAngle, int yAngle, int zAngle)
{
    xSRot += xAngle;
    ySRot += yAngle;
    zSRot += zAngle;
}

void FieldMapDisplay::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void FieldMapDisplay::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastPos.x();	
	int dy = event->y() - lastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		rotateBy(8 * dy, 8 * dx, 0);
		update();
	} else if (event->buttons() & Qt::RightButton) {
		rotateSceneBy(8 * dy, 8 * dy, 8 * dx);
	}
	
	lastPos = event->pos();
}