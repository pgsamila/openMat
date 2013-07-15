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

#include "FieldMapContainer.h"

FieldMapContainer::FieldMapContainer(QWidget* parent) :
	QWidget(parent)
{
	fmd = new FieldMapDisplay();
	fmi = new FieldModuleIndicator();

	QHBoxLayout* h1 = new QHBoxLayout();
	QHBoxLayout* h2 = new QHBoxLayout();
	QVBoxLayout* v1 = new QVBoxLayout();
	QVBoxLayout* v2 = new QVBoxLayout();
	QGridLayout* g1 = new QGridLayout();
	
	v1->addWidget(fmi);
	v1->addWidget(fmd);

	g1->addWidget(new QLabel("Estimated field strength (uT)"), 0, 0);
	g1->addWidget(new QLabel("="), 0, 1);
	g1->addWidget(fieldRadiusLabel = new QLabel("n/a"), 0, 2);

	g1->addWidget(new QLabel("Mag. field offset [x, y, z] (uT)"), 1, 0);
	g1->addWidget(new QLabel("= ["), 1, 1);
	g1->addWidget(fieldOffsetLabel[0] = new QLabel("n/a"), 1, 2);
	g1->addWidget(fieldOffsetLabel[1] = new QLabel(), 1, 3);
	g1->addWidget(fieldOffsetLabel[2] = new QLabel(), 1, 4);
	g1->addWidget(new QLabel("]"), 1, 5);
	
	g1->addWidget(new QLabel("Mag. field distortion [x, y, z] (uT)"), 2, 0);
	g1->addWidget(new QLabel("= ["), 2, 1);
	g1->addWidget(fieldDistortionLabel[0] = new QLabel("n/a"), 2, 2);
	g1->addWidget(fieldDistortionLabel[1] = new QLabel(), 2, 3);
	g1->addWidget(fieldDistortionLabel[2] = new QLabel(), 2, 4);
	g1->addWidget(new QLabel("]"), 2, 5);
	
	h2->addLayout(g1);
	v1->addLayout(h2);
	
	v1->setStretch(0, 1);
	v1->setStretch(1, 10);
	v1->setStretch(2, 1);
	
	setLayout(v1);
}

void FieldMapContainer::updateGraphs(int v)
{
	fmd->showOriginalField = originalFieldCheck->isChecked();
	fmd->showCorrectedField = correctedFieldCheck->isChecked();
	fmd->showEllipsoid = ellipsoidCheck->isChecked();

	fmd->update();
}

void FieldMapContainer::updateCurrentField(float fieldRadius, float currentField[3])
{
	fmi->updateFieldModule(fieldRadius, Eigen::Vector3f(currentField[0], currentField[1], currentField[2]));
}

void FieldMapContainer::updateFieldMap(float magField[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW][3],
	float hardIronOffset[3], float softIronMatrix[3][3],
	float fieldRadius, float currentField[3]) {	
	fmd->fieldRadius = fieldRadius; 

	for (int i=0; i<ABSMAXPITCH; i++) {
		for (int j=0; j<ABSMAXROLL; j++) {
			for (int k=0; k<ABSMAXYAW; k++) {
				for (int l=0; l<3; l++) {
					fmd->fieldMap[i][j][k](l) = magField[i][j][k][l];
				}
			}
		}
	}
	
	for (int i=0; i<3; i++) {
		fmd->hardIronOffset(i) = hardIronOffset[i];
	}
	
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			fmd->softIronMatrix(i, j) = softIronMatrix[i][j];
		}
	}
	
	for (int i=0; i<3; i++) {
		fmd->currentField(i) = currentField[i];
	}
	
	fmd->updateFieldMap();

	fieldRadiusLabel->setText(QString("%1").arg(fmd->fieldRadius, -8, 'f', 2));
	
	fieldOffsetLabel[0]->setText(QString("%1").arg(fmd->hardIronOffset(0), -8, 'f', 2));
	fieldOffsetLabel[1]->setText(QString("%1").arg(fmd->hardIronOffset(1), -8, 'f', 2));
	fieldOffsetLabel[2]->setText(QString("%1").arg(fmd->hardIronOffset(2), -8, 'f', 2));
	
	fieldDistortionLabel[0]->setText(QString("%1").arg(fmd->softIronMatrix(0, 0), -8, 'f', 2));
	fieldDistortionLabel[1]->setText(QString("%1").arg(fmd->softIronMatrix(1, 1), -8, 'f', 2));
	fieldDistortionLabel[2]->setText(QString("%1").arg(fmd->softIronMatrix(2, 2), -8, 'f', 2));
	
	fmd->update();
	fmi->update();
	update();
}