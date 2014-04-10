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
	sagittal_legend = new QwtLegend();
	transverse_legend = new QwtLegend();
	coronal_legend = new QwtLegend();

	sagittal_graph = new Plot("", "Samples", "Sagittal angle (degree)", 
		"X", "", "", "",
		0, 150, 1, 0.0f, 180.0f, sagittal_legend);

	transverse_graph = new Plot("", "Samples", "Transverse angle (degree)", 
		"Y", "", "", "",
		0, 150, 1, 0.0f, 180.0f, transverse_legend);

	coronal_graph = new Plot("", "Samples", "Coronal angle (degree)", 
		"Z", "", "", "",
		0, 150, 1, 0.0f, 180.0f, coronal_legend);

	QVBoxLayout *graphLayout = new QVBoxLayout();

	graphLayout->addWidget(sagittal_graph);
	graphLayout->addWidget(transverse_graph);	
	graphLayout->addWidget(coronal_graph);

	this->setLayout(graphLayout);
	
    setAutoFillBackground(true);
    setPalette(QPalette(QColor(255, 255, 255)));	
}

void GraphWindow::plotData(Eigen::Vector3f planeAngle)
{	
	sagittal_graph->addData(0, planeAngle(0));
	transverse_graph->addData(1, planeAngle(1));
	coronal_graph->addData(2, planeAngle(2));
}

void GraphWindow::clearGraphs(void)
{
	sagittal_graph->clearData();
	transverse_graph->clearData();
	coronal_graph->clearData();
}