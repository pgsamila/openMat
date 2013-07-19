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

#ifndef SAN_ANGELES_WINDOW
#define SAN_ANGELES_WINDOW

#include <QtGui>
#include <QGLWidget>

#include <GL/glu.h>
#include <GL/gl.h>

#include "ImuData.h"
#include "LpmsDefinitions.h"

#include <time.h>

#include <string>
#include <iostream>
using namespace std;

#include <Eigen/Dense>	

class QGLShaderProgram;

typedef struct {
    float x, y, z;
} VECTOR3;

// Definition of one GL object in this demo.
typedef struct {
    /* Vertex array and color array are enabled for all objects, so their
     * pointers must always be valid and non-NULL. Normal array is not
     * used by the ground plane, so when its pointer is NULL then normal
     * array usage is disabled.
     *
     * Vertex array is supposed to use GL_FIXED datatype and stride 0
     * (i.e. tightly packed array). Color array is supposed to have 4
     * components per color with GL_UNSIGNED_BYTE datatype and stride 0.
     * Normal array is supposed to use GL_FIXED datatype and stride 0.
     */
    GLfloat *vertexArray;
    GLubyte *colorArray;
    GLfloat *normalArray;
    GLint vertexComponents;
    GLsizei count;
} GLOBJECT;

#define FIXED(value) floatToFixed(value)

// Total run length is 20 * camera track base unit length (see cams.h).
#define RUN_LENGTH  (20 * CAMTRACK_LEN)
#undef PI
#define PI 3.1415926535897932f
#define RANDOM_UINT_MAX 65535

/*!
	\brief	Shows the 3d-cube visualization of the sampled IMU data.
*/
class SanAngelesWindow : public QGLWidget
{
    Q_OBJECT

public:	
	int activeOpenMatId;
	QPoint lastPos;
	int xRot;
	int yRot;
	int zRot;
	int xSRot;
	int ySRot;
	int zSRot;
	float lpmsCaseScale;
	Eigen::Matrix3f rM;
	ImuData imuData;
	Eigen::Vector3f fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW];
	Eigen::Vector3f hardIronOffset;
	Eigen::Matrix3f softIronMatrix;

    SanAngelesWindow(QWidget *parent = 0, QGLWidget *shareWidget = 0);
    ~SanAngelesWindow();
    QSize minimumSizeHint() const;
    QSize sizeHint() const;
	void setActiveLpms(int openMatId);
    void initializeGL();
    void resizeGL(int width, int height);
	void zeroImuData(ImuData* id);
	
	static void freeGLObject(GLOBJECT *object);

	static GLOBJECT *newGLObject(long vertices, int vertexComponents, int useNormalArray);

	static void drawGLObject(GLOBJECT *object);

	static void vector3Sub(VECTOR3 *dest, VECTOR3 *v1, VECTOR3 *v2);

	static void superShapeMap(VECTOR3 *point, float r1, float r2, float t, float p);

	static float ssFunc(const float t, const float *p);

	// Creates and returns a supershape object.
	// Based on Paul Bourke's POV-Ray implementation.
	// http://astronomy.swin.edu.au/~pbourke/povray/supershape/
	static GLOBJECT *createSuperShape(const float *params);

	static GLOBJECT *createGroundPlane();

	static void drawGroundPlane();

	static void drawFadeQuad();

	// Called from the app framework.
	void appInit();

	// Called from the app framework.
	void appDeinit();

	static void gluPerspective(GLfloat fovy, GLfloat aspect, GLfloat zNear, GLfloat zFar);

	static void prepareFrame(int width, int height);

	static void configureLightAndMaterial();

	static void drawModels(float zScale);

	/* Following gluLookAt implementation is adapted from the
	 * Mesa 3D Graphics library. http://www.mesa3d.org
	 */
	static void gluLookAt(GLfloat eyex, GLfloat eyey, GLfloat eyez, GLfloat centerx, GLfloat centery, GLfloat centerz, GLfloat upx, GLfloat upy, GLfloat upz);

	void camTrack();

	// Called from the app framework.
	/* The tick is current time in milliseconds, width and height
	 * are the image dimensions to be rendered.
	 */
	void appRender(long tick, int width, int height);

	static void seedRandom(unsigned long seed);
	
	static unsigned long randomUInt();

	// Capped conversion from float to fixed.
	static long floatToFixed(float value);

signals:
    void clicked();

public slots:	
	void updateQuaternion(ImuData imuData);
	void updateWindow(void);
	void rotateBy(int xAngle, int yAngle, int zAngle);
	void rotateSceneBy(int xAngle, int yAngle, int zAngle);
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);	
    void paintGL();	
};

#endif