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

/* San Angeles Observation OpenGL ES version example
 * Copyright 2004-2005 Jetro Lauha
 * All rights reserved.
 * Web: http://iki.fi/jetro/
 *
 * This source is free software; you can redistribute it and/or
 * modify it under the terms of EITHER:
 *   (1) The GNU Lesser General Public License as published by the Free
 *       Software Foundation; either version 2.1 of the License, or (at
 *       your option) any later version. The text of the GNU Lesser
 *       General Public License is included with this source in the
 *       file LICENSE-LGPL.txt.
 *   (2) The BSD-style license that is included with this source in
 *       the file LICENSE-BSD.txt.
 *
 * This source is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 * LICENSE-LGPL.txt and LICENSE-BSD.txt for more details.
 *
 * $Id: shapes.h,v 1.6 2005/01/31 22:15:30 tonic Exp $
 * $Revision: 1.6 $
 */

#include "SanAngelesWindow.h"

#include <iostream>
using namespace std;

#include "shapes.h"
#include "cams.h"

static long sStartTick = 0;
static long sTick = 0;
static int sCurrentCamTrack = 0;
static long sCurrentCamTrackStartTick = 0;
static long sNextCamTrackStartTick = 0x7fffffff;
static GLOBJECT *sSuperShapeObjects[SUPERSHAPE_COUNT] = { NULL };
static GLOBJECT *sGroundPlane = NULL;
const float d2r = 0.01745f;	
static unsigned long sRandomSeed = 0;
int objIndex[20][20];
long passedTime = 0;
long timeInc = 25;

SanAngelesWindow::SanAngelesWindow(QWidget *parent, QGLWidget *shareWidget)
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
	
	zeroImuData(&imuData);
	
	rM = Eigen::Matrix3f::Identity();
}

void SanAngelesWindow::zeroImuData(ImuData* id)
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

void SanAngelesWindow::updateWindow(void)
{
	passedTime += timeInc;
	
	updateGL();
}

SanAngelesWindow::~SanAngelesWindow()
{
	appDeinit();
}

QSize SanAngelesWindow::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize SanAngelesWindow::sizeHint() const
{
    return QSize(200, 200);
}

void SanAngelesWindow::initializeGL()
{
	appInit();
}

void SanAngelesWindow::updateQuaternion(ImuData imuData)
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
				0, 1, 0,
				0, 0, 1;	
				
	M = Eigen::AngleAxisf(90 * d2r, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(-90 * d2r, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(-90 * d2r, Eigen::Vector3f::UnitZ());
		
	rM = M * (tM * scale);
}

void SanAngelesWindow::paintGL()
{
	appRender(passedTime, width(), height());
}

void SanAngelesWindow::resizeGL(int width, int height)
{
}

void SanAngelesWindow::setActiveLpms(int openMatId)
{
	activeOpenMatId = openMatId;
}

void SanAngelesWindow::rotateBy(int xAngle, int yAngle, int zAngle)
{
    xRot += xAngle;
    yRot += yAngle;
    zRot += zAngle;
	
    updateGL();
}

void SanAngelesWindow::rotateSceneBy(int xAngle, int yAngle, int zAngle)
{
    xSRot += xAngle;
    ySRot += yAngle;
    zSRot += zAngle;
	
    updateGL();
}

void SanAngelesWindow::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastPos.x();	
	int dy = event->y() - lastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		rotateBy(8 * dy, 0, 8 * dx);
	} else if (event->buttons() & Qt::RightButton) {
		rotateSceneBy(8 * dy, 8 * dy, 8 * dx);
	}
	
	lastPos = event->pos();
}

void SanAngelesWindow::freeGLObject(GLOBJECT *object)
{
    if (object == NULL) return;
	
    free(object->normalArray);
    free(object->colorArray);
    free(object->vertexArray);
    free(object);
}

GLOBJECT * SanAngelesWindow::newGLObject(long vertices, int vertexComponents, int useNormalArray)
{
    GLOBJECT *result;
    result = (GLOBJECT *)malloc(sizeof(GLOBJECT));
    if (result == NULL)
        return NULL;
    result->count = vertices;
    result->vertexComponents = vertexComponents;
    result->vertexArray = (GLfloat *)malloc(vertices * vertexComponents * sizeof(GLfloat));
    result->colorArray = (GLubyte *)malloc(vertices * 4 * sizeof(GLubyte));
	
    if (useNormalArray) {
        result->normalArray = (GLfloat *)malloc(vertices * 3 * sizeof(GLfloat));
    } else {
		result->normalArray = NULL;
	}
	
    if (result->vertexArray == NULL ||
		result->colorArray == NULL ||
		(useNormalArray && result->normalArray == NULL)) {
		freeGLObject(result);
		return NULL;
    }
	
	return result;
}


void SanAngelesWindow::drawGLObject(GLOBJECT *object)
{
    assert(object != NULL);

    glVertexPointer(object->vertexComponents, GL_FLOAT,
					0, object->vertexArray);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, object->colorArray);

    if (object->normalArray) {
        glNormalPointer(GL_FLOAT, 0, object->normalArray);
        glEnableClientState(GL_NORMAL_ARRAY);
    } else {
        glDisableClientState(GL_NORMAL_ARRAY);
	}
	
	glDrawArrays(GL_TRIANGLES, 0, object->count);
}


void SanAngelesWindow::vector3Sub(VECTOR3 *dest, VECTOR3 *v1, VECTOR3 *v2)
{
	dest->x = v1->x - v2->x;
	dest->y = v1->y - v2->y;
	dest->z = v1->z - v2->z;
}

void SanAngelesWindow::superShapeMap(VECTOR3 *point, float r1, float r2, float t, float p)
{
	// sphere-mapping of supershape parameters
	point->x = (float)(cos((float)t) * cos((float)p) / r1 / r2);
	point->y = (float)(sin((float)t) * cos((float)p) / r1 / r2);
	point->z = (float)(sin((float)p) / r2);
}

float SanAngelesWindow::ssFunc(const float t, const float *p)
{
    return (float)(pow(pow(fabs(cos((float)p[0] * t / 4)) / p[1], p[4]) + pow(fabs(sin((float)p[0] * t / 4)) / p[2], p[5]), 1 / p[3]));
}

// Creates and returns a supershape object.
// Based on Paul Bourke's POV-Ray implementation.
// http://astronomy.swin.edu.au/~pbourke/povray/supershape/
GLOBJECT * SanAngelesWindow::createSuperShape(const float *params)
{
    const int resol1 = (float)params[SUPERSHAPE_PARAMS - 3];
    const int resol2 = (float)params[SUPERSHAPE_PARAMS - 2];
    // latitude 0 to pi/2 for no mirrored bottom
    // (latitudeBegin==0 for -pi/2 to pi/2 originally)
    int latitudeBegin = resol2 / 4;
    int latitudeEnd = resol2 / 2; // non-inclusive
    const int longitudeCount = resol1;
    const int latitudeCount = latitudeEnd - latitudeBegin;
    const long triangleCount = longitudeCount * latitudeCount * 2;
    const long vertices = triangleCount * 3;
    GLOBJECT *result;
    float baseColor[3];
    int a, longitude, latitude;
    long currentVertex, currentQuad;

    result = newGLObject(vertices, 3, 1);
    if (result == NULL) return NULL;

    for (a = 0; a < 3; ++a) {
		baseColor[a] = ((rand() % 155) + 100) / 255.f;
	}

    currentQuad = 0;
    currentVertex = 0;

    // longitude -pi to pi
    for (longitude = 0; longitude < longitudeCount; ++longitude) {
        // latitude 0 to pi/2
        for (latitude = latitudeBegin; latitude < latitudeEnd; ++latitude) {
            float t1 = -PI + longitude * 2 * PI / resol1;
            float t2 = -PI + (longitude + 1) * 2 * PI / resol1;
            float p1 = -PI / 2 + latitude * 2 * PI / resol2;
            float p2 = -PI / 2 + (latitude + 1) * 2 * PI / resol2;
            float r0, r1, r2, r3;

            r0 = ssFunc(t1, params);
            r1 = ssFunc(p1, &params[6]);
            r2 = ssFunc(t2, params);
            r3 = ssFunc(p2, &params[6]);

            if (r0 != 0 && r1 != 0 && r2 != 0 && r3 != 0) {
                VECTOR3 pa, pb, pc, pd;
                VECTOR3 v1, v2, n;
                float ca;
                int i;
                // float lenSq, invLenSq;

                superShapeMap(&pa, r0, r1, t1, p1);
                superShapeMap(&pb, r2, r1, t2, p1);
                superShapeMap(&pc, r2, r3, t2, p2);
                superShapeMap(&pd, r0, r3, t1, p2);

                // kludge to set lower edge of the object to fixed level
                if (latitude == latitudeBegin + 1)
                    pa.z = pb.z = 0;

                vector3Sub(&v1, &pb, &pa);
                vector3Sub(&v2, &pd, &pa);

                // Calculate normal with cross product.
				/*   i    j    k      i    j
				* v1.x v1.y v1.z | v1.x v1.y
				* v2.x v2.y v2.z | v2.x v2.y
				*/

                n.x = v1.y * v2.z - v1.z * v2.y;
                n.y = v1.z * v2.x - v1.x * v2.z;
                n.z = v1.x * v2.y - v1.y * v2.x;

				/* Pre-normalization of the normals is disabled here because
				* they will be normalized anyway later due to automatic
				* normalization (GL_NORMALIZE). It is enabled because the
				* objects are scaled with glScale.
				*/

				/* lenSq = n.x * n.x + n.y * n.y + n.z * n.z;
				invLenSq = (float)(1 / sqrt(lenSq));
				n.x *= invLenSq;
				n.y *= invLenSq;
				n.z *= invLenSq; */

                ca = pa.z + 0.5f;

                for (i = currentVertex * 3;
                     i < (currentVertex + 6) * 3;
                     i += 3) {
                    result->normalArray[i] = n.x;
                    result->normalArray[i + 1] = n.y;
                    result->normalArray[i + 2] = n.z;
                }
				
                for (i = currentVertex * 4; i < (currentVertex + 6) * 4; i += 4) {
					int a, color[3];
                    
					for (a = 0; a < 3; ++a) {
						color[a] = (int)(ca * baseColor[a] * 255);
						if (color[a] > 255) color[a] = 255;
					}
                    
					result->colorArray[i] = (GLubyte)color[0];
                    result->colorArray[i + 1] = (GLubyte)color[1];
                    result->colorArray[i + 2] = (GLubyte)color[2];
                    result->colorArray[i + 3] = 0;
				}
                
				result->vertexArray[currentVertex * 3] = pa.x;
                result->vertexArray[currentVertex * 3 + 1] = pa.y;
                result->vertexArray[currentVertex * 3 + 2] = pa.z;
                
				++currentVertex;
                
				result->vertexArray[currentVertex * 3] = pb.x;
                result->vertexArray[currentVertex * 3 + 1] = pb.y;
                result->vertexArray[currentVertex * 3 + 2] = pb.z;
                
				++currentVertex;
                
				result->vertexArray[currentVertex * 3] = pd.x;
                result->vertexArray[currentVertex * 3 + 1] = pd.y;
                result->vertexArray[currentVertex * 3 + 2] = pd.z;
                
				++currentVertex;
                
				result->vertexArray[currentVertex * 3] = pb.x;
                result->vertexArray[currentVertex * 3 + 1] = pb.y;
                result->vertexArray[currentVertex * 3 + 2] = pb.z;
                
				++currentVertex;
                
				result->vertexArray[currentVertex * 3] = pc.x;
                result->vertexArray[currentVertex * 3 + 1] = pc.y;
                result->vertexArray[currentVertex * 3 + 2] = pc.z;
                
				++currentVertex;
                
				result->vertexArray[currentVertex * 3] = pd.x;
                result->vertexArray[currentVertex * 3 + 1] = pd.y;
                result->vertexArray[currentVertex * 3 + 2] = pd.z;
                
				++currentVertex;
            } // r0 && r1 && r2 && r3

			++currentQuad;
        } // latitude
    } // longitude

    // Set number of vertices in object to the actual amount created
	result->count = currentVertex;

	return result;
}

GLOBJECT * SanAngelesWindow::createGroundPlane()
{
	const int scale = 4;
	const int yBegin = -15, yEnd = 15;    // ends are non-inclusive
	const int xBegin = -15, xEnd = 15;
	const long triangleCount = (yEnd - yBegin) * (xEnd - xBegin) * 2;
	const long vertices = triangleCount * 3;
	GLOBJECT *result;
	int x, y;
	long currentVertex, currentQuad;

    result = newGLObject(vertices, 2, 0);
    if (result == NULL) {
		return NULL;
	}

    currentQuad = 0;
    currentVertex = 0;

    for (y = yBegin; y < yEnd; ++y) {
        for (x = xBegin; x < xEnd; ++x) {
			GLubyte color;
			int i, a;
			color = (GLubyte)((randomUInt() & 0x5f) + 81);  // 101 1111
			for (i = currentVertex * 4; i < (currentVertex + 6) * 4; i += 4) {
                result->colorArray[i] = color;
                result->colorArray[i + 1] = color;
                result->colorArray[i + 2] = color;
                result->colorArray[i + 3] = 0;
            }

            // Axis bits for quad triangles:
            // x: 011100 (0x1c), y: 110001 (0x31)  (clockwise)
            // x: 001110 (0x0e), y: 100011 (0x23)  (counter-clockwise)
            for (a = 0; a < 6; ++a)
            {
                const int xm = x + ((0x1c >> a) & 1);
                const int ym = y + ((0x31 >> a) & 1);
                const float m = (float)(cos((float)xm * 2) * sin((float)ym * 4) * 0.75f);
                result->vertexArray[currentVertex * 2] =
                    xm * scale + m;
                result->vertexArray[currentVertex * 2 + 1] =
                    ym * scale + m;
                ++currentVertex;
            }
            ++currentQuad;
        }
    }
    return result;
}

void SanAngelesWindow::drawGroundPlane()
{
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ZERO, GL_SRC_COLOR);
    glDisable(GL_LIGHTING);

    drawGLObject(sGroundPlane);

    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

void SanAngelesWindow::drawFadeQuad()
{
}

// Called from the app framework.
void SanAngelesWindow::appInit()
{
    int a;
	bool f = false;
	int curShape;

    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    seedRandom(20);
	srand((unsigned)time(0));

    for (a = 0; a < SUPERSHAPE_COUNT; ++a) {
		sSuperShapeObjects[a] = createSuperShape(sSuperShapeParams[a]);
		assert(sSuperShapeObjects[a] != NULL);
	}
	
	for (int y = -5; y <= 5; ++y) {
		for (int x = -5; x <= 5; ++x) {			
			f = false;
			while (f == false) {
				curShape = rand() % SUPERSHAPE_COUNT;
				if (curShape == 0 || 
					curShape == 1 || 
					curShape == 3 || 
					curShape == 6 || 
					curShape == 10 || 					
					curShape == 11 ||
					curShape == 13 ||
					curShape == 14 ||
					curShape == 17 ||
					curShape == 19) {
					f = false;
				} else {
					f = true;
				}
			}
			// curShape = 15;
			objIndex[x+5][y+5] = curShape;			
		}
	}
    
	sGroundPlane = createGroundPlane();
    assert(sGroundPlane != NULL);
}

// Called from the app framework.
void SanAngelesWindow::appDeinit()
{
    int a;
	
    for (a = 0; a < SUPERSHAPE_COUNT; ++a) {
		freeGLObject(sSuperShapeObjects[a]);
	}
	
	freeGLObject(sGroundPlane);
}

void SanAngelesWindow::gluPerspective(GLfloat fovy, GLfloat aspect,
	GLfloat zNear, GLfloat zFar)
{
	GLfloat xmin, xmax, ymin, ymax;

	ymax = zNear * (GLfloat)tan(fovy * PI / 360);
	ymin = -ymax;
	xmin = ymin * aspect;
	xmax = ymax * aspect;

	glFrustum((GLfloat)(xmin), (GLfloat)(xmax),
		(GLfloat)(ymin), (GLfloat)(ymax),
		(GLfloat)(zNear), (GLfloat)(zFar));
}

void SanAngelesWindow::prepareFrame(int width, int height)
{
	glViewport(0, 0, width, height);

	glClearColor((GLfloat)(0.1f),
		(GLfloat)(0.2f),
		(GLfloat)(0.3f), (float) 0x10000 / 65536.0f);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, (float)width / height, 0.3f, 250);

	glMatrixMode(GL_MODELVIEW);

	glLoadIdentity();
}

void SanAngelesWindow::configureLightAndMaterial()
{
	static GLfloat light0Position[] = { -4.0f, 1.0f, 1.0f, 0 };
	static GLfloat light0Diffuse[] = { 1.0f, (float) 0x6666 / 65536.0f, 0, 1.0f };
	static GLfloat light1Position[] = { 1.0f, -2.0f, 1.0f, 0 };
	static GLfloat light1Diffuse[] = { (float) 0x11eb / 65536.0f, (float) 0x23d7 / 65536.0f, (float) 0x5999 / 65536.0f, 1.0f };
	static GLfloat light2Position[] = { -1.0f, 0, -4.0f, 0 };
	static GLfloat light2Diffuse[] = { (float) 0x11eb / 65536.0f, (float) 0x2b85 / 65536.0f, (float) 0x23d7 / 65536.0f, 1.0f / 65536.0f };
	static GLfloat materialSpecular[] = { 1.0f, 1.0f, 1.0f, 1.0f };

    glLightfv(GL_LIGHT0, GL_POSITION, light0Position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0Diffuse);
    glLightfv(GL_LIGHT1, GL_POSITION, light1Position);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light1Diffuse);
    glLightfv(GL_LIGHT2, GL_POSITION, light2Position);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, light2Diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, materialSpecular);

    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 60);
    glEnable(GL_COLOR_MATERIAL);
}

void SanAngelesWindow::drawModels(float zScale)
{
    const int translationScale = 9;
    int x, y;

    seedRandom(9);

    glScalef(1, 1, (GLfloat)(zScale));

	for (y = -5; y <= 5; ++y) {
		for (x = -5; x <= 5; ++x) {
			if (!(x == 0 && y == 0)) { 
				float buildingScale;
				GLfloat fixedScale;

				int curShape = objIndex[x+5][y+5];
							
				buildingScale = sSuperShapeParams[curShape][SUPERSHAPE_PARAMS - 1];
				fixedScale = (GLfloat)(buildingScale);

				glPushMatrix();
				glTranslatef((x * translationScale),
					(y * translationScale),
					0);
				glRotatef((GLfloat)((randomUInt() % 360)), 0, 0, 1);
				glScalef(fixedScale, fixedScale * 0.75, fixedScale * 2);

				drawGLObject(sSuperShapeObjects[curShape]);
				glPopMatrix();
			}
		}
	}

    for (x = -2; x <= 2; ++x) {
		const int shipScale100 = translationScale * 500;
		const int offs100 = x * shipScale100 + (sTick % shipScale100);
		float offs = offs100 * 0.01f;
		GLfloat fixedOffs = (GLfloat)(offs);
		glPushMatrix();
		glTranslatef(fixedOffs, -4, 2);
		drawGLObject(sSuperShapeObjects[SUPERSHAPE_COUNT - 1]);
		glPopMatrix();
		glPushMatrix();
		glTranslatef(-4, fixedOffs, 4);
		glRotatef(90, 0, 0, 1);
		drawGLObject(sSuperShapeObjects[SUPERSHAPE_COUNT - 1]);
		glPopMatrix();
    }
}

/* Following gluLookAt implementation is adapted from the
 * Mesa 3D Graphics library. http://www.mesa3d.org
 */
void SanAngelesWindow::gluLookAt(GLfloat eyex, GLfloat eyey, GLfloat eyez,
				GLfloat centerx, GLfloat centery, GLfloat centerz,
				GLfloat upx, GLfloat upy, GLfloat upz)
{
    GLfloat m[16];
    GLfloat x[3], y[3], z[3];
    GLfloat mag;

    /* Make rotation matrix */

    /* Z vector */
    z[0] = eyex - centerx;
    z[1] = eyey - centery;
    z[2] = eyez - centerz;
    mag = (float)sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
    if (mag) {			/* mpichler, 19950515 */
		z[0] /= mag;
		z[1] /= mag;
		z[2] /= mag;
    }

    /* Y vector */
    y[0] = upx;
    y[1] = upy;
    y[2] = upz;

    /* X vector = Y cross Z */
    x[0] = y[1] * z[2] - y[2] * z[1];
    x[1] = -y[0] * z[2] + y[2] * z[0];
    x[2] = y[0] * z[1] - y[1] * z[0];

    /* Recompute Y = Z cross X */
    y[0] = z[1] * x[2] - z[2] * x[1];
    y[1] = -z[0] * x[2] + z[2] * x[0];
    y[2] = z[0] * x[1] - z[1] * x[0];

    /* mpichler, 19950515 */
    /* cross product gives area of parallelogram, which is < 1.0 for
     * non-perpendicular unit-length vectors; so normalize x, y here
     */

    mag = (float)sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    if (mag) {
        x[0] /= mag;
        x[1] /= mag;
        x[2] /= mag;
    }

    mag = (float)sqrt(y[0] * y[0] + y[1] * y[1] + y[2] * y[2]);
    if (mag) {
        y[0] /= mag;
        y[1] /= mag;
        y[2] /= mag;
    }

#define M(row,col)  m[col*4+row]
    M(0, 0) = x[0];
    M(0, 1) = x[1];
    M(0, 2) = x[2];
    M(0, 3) = 0.0;
    M(1, 0) = y[0];
    M(1, 1) = y[1];
    M(1, 2) = y[2];
    M(1, 3) = 0.0;
    M(2, 0) = z[0];
    M(2, 1) = z[1];
    M(2, 2) = z[2];
    M(2, 3) = 0.0;
    M(3, 0) = 0.0;
    M(3, 1) = 0.0;
    M(3, 2) = 0.0;
    M(3, 3) = 1.0;
#undef M
    {
        int a;
        GLfloat fixedM[16];
        for (a = 0; a < 16; ++a)
            fixedM[a] = (GLfloat)(m[a]);
        glMultMatrixf(fixedM);
    }

    /* Translate Eye to Origin */
    glTranslatef((GLfloat)(-eyex),
                 (GLfloat)(-eyey),
                 (GLfloat)(-eyez));
}

void SanAngelesWindow::camTrack()
{
    float lerp[5];
    float eX, eY, eZ, cX, cY, cZ;
    float trackPos;
    CAMTRACK *cam;
    long currentCamTick;
    int a;

    if (sNextCamTrackStartTick <= sTick) {
        ++sCurrentCamTrack;
        sCurrentCamTrackStartTick = sNextCamTrackStartTick;
    }
	
    sNextCamTrackStartTick = sCurrentCamTrackStartTick +
                             sCamTracks[sCurrentCamTrack].len * CAMTRACK_LEN;

    cam = &sCamTracks[sCurrentCamTrack];
    currentCamTick = sTick - sCurrentCamTrackStartTick;
    trackPos = (float)currentCamTick / (CAMTRACK_LEN * cam->len);

    for (a = 0; a < 5; ++a) {
        lerp[a] = (cam->src[a] + cam->dest[a] * trackPos) * 0.01f;
	}

    if (cam->dist) {
        float dist = cam->dist * 0.1f;
        cX = lerp[0];
        cY = lerp[1];
        cZ = lerp[2];
        eX = cX - (float)cos((float)lerp[3]) * dist;
        eY = cY - (float)sin((float)lerp[3]) * dist;
        eZ = cZ - lerp[4];
    } else {
        eX = lerp[0];
        eY = lerp[1];
        eZ = lerp[2];
        cX = eX + (float)cos((float)lerp[3]);
        cY = eY + (float)sin((float)lerp[3]);
        cZ = eZ + lerp[4];
    }
    
	gluLookAt(eX, eY, eZ, cX, cY, cZ, 0, 0, 1);
}

// Called from the app framework.
/* The tick is current time in milliseconds, width and height
 * are the image dimensions to be rendered.
 */
void SanAngelesWindow::appRender(long tick, int width, int height)
{
    GLfloat m[16];
	const float d2r = 0.01745f;
	const float r2d = 57.2958f;	

    if (sStartTick == 0) sStartTick = tick;

    // Actual tick value is "blurred" a little bit.
    sTick = (sTick + tick - sStartTick) >> 1;

    // Prepare OpenGL ES for rendering of the frame.
    prepareFrame(width, height);

    // Update the camera position and set the lookat.
    // camTrack();	
	
	glLoadIdentity();	
	
	/* if (imuData.r[1] > 45) imuData.r[1] = 45;
	if (imuData.r[1] < -45) imuData.r[1] = -45;		
	
	if (imuData.r[0] > 90) imuData.r[0] = 90;
	if (imuData.r[0] < -90) imuData.r[0] = -90;		
	
	glRotatef(-imuData.r[1], 0.0f, 0.0f, 1.0f);
	glRotatef(imuData.r[0] - 90.0, 1.0f, 0.0f, 0.0f);
	glRotatef(-imuData.r[2], 0.0f, 0.0f, 1.0f);	*/
	
	#define M(row,col)  m[col*4+row]
		M(0, 0) = rM(0, 0);
		M(0, 1) = rM(0, 1);
		M(0, 2) = rM(0, 2);
		M(0, 3) = 0.0;
		M(1, 0) = rM(1, 0);
		M(1, 1) = rM(1, 1);
		M(1, 2) = rM(1, 2);
		M(1, 3) = 0.0;
		M(2, 0) = rM(2, 0);
		M(2, 1) = rM(2, 1);
		M(2, 2) = rM(2, 2);
		M(2, 3) = 0.0;
		M(3, 0) = 0.0;
		M(3, 1) = 0.0;
		M(3, 2) = 0.0;
		M(3, 3) = 1.0;
	#undef M
		{
			int a;
			GLfloat fixedM[16];
			for (a = 0; a < 16; ++a)
				fixedM[a] = (GLfloat)(m[a]);
			glMultMatrixf(fixedM);
		}
	
    glTranslatef(0.0f, 0.0f, -7.5f);	
	
    /* glRotatef(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
	glRotatef(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
	glRotatef(zRot / 16.0f, 0.0f, 0.0f, 1.0f);	*/
	
    // Configure environment.
    configureLightAndMaterial();

    // Draw the reflection by drawing models with negated Z-axis.
    glPushMatrix();
    drawModels(-1);
    glPopMatrix();

    // Blend the ground plane to the window.
    drawGroundPlane();

    // Draw all the models normally.
    drawModels(1);
	
    glPopMatrix();
	
    // Draw fade quad over whole window (when changing cameras).
	// drawFadeQuad();
}

void SanAngelesWindow::seedRandom(unsigned long seed)
{
    sRandomSeed = seed;
}

unsigned long SanAngelesWindow::randomUInt()
{
    sRandomSeed = sRandomSeed * 0x343fd + 0x269ec3;
    return sRandomSeed >> 16;
}

void SanAngelesWindow::mousePressEvent(QMouseEvent *event)
{
	if (parentWidget()->isFullScreen() == true) {
		parentWidget()->showNormal();
	} else {
		parentWidget()->showFullScreen();
	}	
}