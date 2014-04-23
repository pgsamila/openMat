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
	sagittal_graph = new Plot("title", "Samples", "Angle (degree)", "S", "T", "C", "", 0, 150, 3, 0.0f, 180.0f, sagittal_legend, this);

	QVBoxLayout *graphLayout = new QVBoxLayout();

	graphLayout->addWidget(sagittal_graph);

	this->setLayout(graphLayout);
	
    setAutoFillBackground(true);
    setPalette(QPalette(QColor(255, 255, 255)));
}

void GraphWindow::plotData(double x, double y, double z)
{	
	sagittal_graph->addData(0, x);
	sagittal_graph->addData(1, y);
	sagittal_graph->addData(2, z);
}

void GraphWindow::clearGraphs(void)
{
	// sagittal_graph->clearData();
}