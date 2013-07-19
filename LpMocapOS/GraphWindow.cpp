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

#include "GraphWindow.h"

GraphWindow::GraphWindow(QWidget* parent) : QWidget(parent)
{
	sagittalGraph = new Plot("Sagittal plane angle (degree)", "No. samples", "", 1, 150, 1, 0, 180);	
	transverseGraph = new Plot("Transverse plane angle (degree)", "No. samples", "", 2, 150, 1, 0, 180);	
	coronalGraph = new Plot("Coronal plane angle (degree)", "No. samples", "", 3, 150, 1, 0, 180);	
	
	QVBoxLayout *graphLayout = new QVBoxLayout();
	graphLayout->addWidget(sagittalGraph);
	graphLayout->addWidget(transverseGraph);
	graphLayout->addWidget(coronalGraph);

	this->setLayout(graphLayout);
	
    setAutoFillBackground(true);
    setPalette(QPalette(QColor(255, 255, 255)));	
}

void GraphWindow::plotData(Eigen::Vector3f planeAngle)
{	
	sagittalGraph->addData(0, planeAngle(0));
	transverseGraph->addData(0, planeAngle(1));
	coronalGraph->addData(0, planeAngle(2));
}

void GraphWindow::clearGraphs(void)
{
	sagittalGraph->clearData();
	transverseGraph->clearData();
	coronalGraph->clearData();
}