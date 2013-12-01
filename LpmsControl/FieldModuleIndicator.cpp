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

#include "FieldModuleIndicator.h"

FieldModuleIndicator::FieldModuleIndicator(QWidget* parent) : 
	QWidget(parent) {
	setMinimumHeight(30);
	setMinimumWidth(200);
	
	tFN = 0.0f;
	cFN = 0.0f;
}

void FieldModuleIndicator::updateFieldModule(float m, Eigen::Vector3f b)
{
	if (m < 0 || m > 1.0e4f) return; 

	float d = fabs(m - b.norm());
	cFN = 0.9 * cFN + 0.1 * m; 
	
	if (cFN > 100) {
		tFN = 1.0;
	} else {
		tFN = cFN / 100.0f;
	}
	
	update();
}

#define EPSILON_INCLINATION 10.0f
#define EPSILON_MODULE 10.0f

void FieldModuleIndicator::paintEvent(QPaintEvent*)
{
	QPainter painter(this);

	QFont f;
	f.setPixelSize(15);
	painter.setFont(f);
	
	float sFN = tFN * (width()-100-7);
	float col = tFN * 255;
	
	QPen white(QColor(255, 255, 255));
	QPen black(QColor(80, 80, 80));
	QPen red(QColor(255, 150, 150));
	QBrush redB(QColor(255, 150, 150));
	QPen blue(QColor(150, 150, 255));
	QBrush blueB(QColor(150, 150, 255));
	QBrush whiteB(QColor(240, 240, 240));
	
	painter.setPen(black);
	painter.setBrush(whiteB);
	painter.drawText(0, height() / 2 - 10, QString("Magnetic"));
	painter.drawText(0, height() / 2 + 10, QString("noise:"));
	painter.drawRect(100, 0, width()-105, height()-5);
	
	painter.setPen(blue);	
	painter.setBrush(blueB);
	painter.drawRect(101, 1, (int) sFN, height()-7);	

	painter.setPen(black);	
	painter.drawText(150, height() / 2, QString("weak"));		
	painter.drawText(width() - 100, height() / 2, QString("strong"));	
}