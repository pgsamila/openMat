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

#include "GraphWindow.h"

#include <iostream>
#include <string>

using namespace std;

GraphWindow::GraphWindow(QWidget* parent) : QWidget(parent)
{
	accLegend = new QwtLegend();
	gyroLegend = new QwtLegend();
	magLegend = new QwtLegend();
	quaternionLegend = new QwtLegend();
	angleLegend = new QwtLegend();
	pressureLegend = new QwtLegend();
	altitudeLegend = new QwtLegend();	
	temperatureLegend = new QwtLegend();
	linAccLegend = new QwtLegend();
	heaveMotionLegend = new QwtLegend();

    accGraph = new Plot("", "Samples", "Accelerometer (g)", 
		"X", "Y", "Z", "",
		0, 500, 3, -8.0f, 8.0f, accLegend);	
		
    gyroGraph = new Plot("", "Samples", "Gyroscope (deg / s)", 
		"X", "Y", "Z", "",
		0, 500, 3, -2000.0f, 2000.0f, gyroLegend);	
		
    magGraph = new Plot("", "Samples", "Magnetometer (uT)", 
		"X", "Y", "Z", "",		
		0, 500, 3, -200, 200, magLegend);	
		
    quaternionGraph = new Plot("", "Samples", "Quaternion (norm)", 
		"W", "X", "Y", "Z",			
		0, 500, 4, -1.0f, 1.0f, quaternionLegend);	
		
    angleGraph = new Plot("", "Samples", "Euler angle (degree)", 
		"X", "Y", "Z", "",			
		0, 500, 3, -181.0f, 181.0f, angleLegend);	
	
	pressureGraph = new Plot("", "Samples", "Bar. Pressure (kPa)", 
		"P", "", "", "", 
		0, 500, 1, 0.0f, 200.0f, pressureLegend);	

	altitudeGraph = new Plot("", "Samples", "Altitude (m)", 
		"A", "", "", "", 
		0, 500, 1, -50.0f, 50.0f, altitudeLegend);	
		
	temperatureGraph = new Plot("", "Samples", "Temperature (deg. C)", 
		"T", "", "", "",
		0, 500, 1, -50.0f, 50.0f, temperatureLegend);
    
	linAccGraph = new Plot("", "Samples", "Linear acceleration (g)", 
		"X", "Y", "Z", "",	
		0, 500, 3, -2.5f, 2.5f, linAccLegend);
		
	heaveMotionGraph = new Plot("", "Samples", "Heave height (m)", 
		"h", "", "", "",
		0, 500, 1, -1.0f, 1.0f, heaveMotionLegend);
	
	QGridLayout *vLayout = new QGridLayout();
	
	vLayout->addWidget(accGraph, 0, 0);	
	vLayout->addWidget(gyroGraph, 1, 0);	
	vLayout->addWidget(magGraph, 2, 0);
	vLayout->addWidget(quaternionGraph, 3, 0);
	vLayout->addWidget(angleGraph, 4, 0);
	vLayout->addWidget(linAccGraph, 5, 0);		
	vLayout->addWidget(pressureGraph, 6, 0);
	vLayout->addWidget(altitudeGraph, 7, 0);
	vLayout->addWidget(temperatureGraph, 8, 0);
	vLayout->addWidget(heaveMotionGraph, 9, 0);
	vLayout->setColumnStretch(0, 2);

	this->setLayout(vLayout);
		
    setAutoFillBackground(true);
    setPalette(QPalette(QColor(255, 255, 255)));

	setMode(GRAPH_MODE_RAW);
}

void GraphWindow::setMode(int mode)
{
	this->mode = mode;

	clearGraphs();

	switch (mode) {
	case GRAPH_MODE_RAW:
		quaternionGraph->hide();
		angleGraph->hide();
		pressureGraph->hide();
		linAccGraph->hide();
		altitudeGraph->hide();
		temperatureGraph->hide();
		heaveMotionGraph->hide();
		
		quaternionLegend->hide();	
		angleLegend->hide();	
		pressureLegend->hide();	
		linAccLegend->hide();	
		altitudeLegend->hide();
		temperatureLegend->hide();	
		heaveMotionLegend->hide();

		accLegend->show();
		accGraph->show();

		gyroGraph->show();
		gyroLegend->show();
		
		magGraph->show();
		magLegend->show();
	break;
		
	case GRAPH_MODE_ORIENTATION:
		accGraph->hide();
		gyroGraph->hide();
		magGraph->hide();
		pressureGraph->hide();
		altitudeGraph->hide();
		temperatureGraph->hide();
		heaveMotionGraph->hide();
		
		accLegend->hide();	
		gyroLegend->hide();
		magLegend->hide();	
		pressureLegend->hide();	
		altitudeLegend->hide();
		temperatureLegend->hide();
		heaveMotionLegend->hide();

		quaternionLegend->show();	
		quaternionGraph->show();
		
		linAccGraph->show();
		linAccLegend->show();

		angleLegend->show();	
		angleGraph->show();		
	break;
		
	case GRAPH_MODE_PRESSURE:		
		quaternionGraph->hide();
		angleGraph->hide();
		accGraph->hide();
		gyroGraph->hide();
		magGraph->hide();
		linAccGraph->hide();
		heaveMotionGraph->hide();
			
		quaternionLegend->hide();	
		angleLegend->hide();	
		accLegend->hide();	
		gyroLegend->hide();
		magLegend->hide();	
		linAccLegend->hide();
		heaveMotionLegend->hide();

		pressureLegend->show();
		pressureGraph->show();	
		
		altitudeGraph->show();
		altitudeLegend->show();

		temperatureGraph->show();
		temperatureLegend->show();
	break;
	
	case GRAPH_MODE_HEAVEMOTION:		
		quaternionGraph->hide();
		angleGraph->hide();
		accGraph->hide();
		gyroGraph->hide();
		magGraph->hide();
		linAccGraph->hide();
		heaveMotionGraph->hide();
		pressureLegend->hide();
		altitudeGraph->hide();
		temperatureGraph->hide();
		
		quaternionLegend->hide();	
		angleLegend->hide();	
		accLegend->hide();	
		gyroLegend->hide();
		magLegend->hide();	
		linAccLegend->hide();
		heaveMotionLegend->hide();
		pressureGraph->hide();
		altitudeLegend->hide();
		temperatureLegend->hide();
		
		heaveMotionGraph->show();
		heaveMotionLegend->show();
	break;
	}	
}

void GraphWindow::plotDataSet(ImuData ld)
{
	switch (mode) {
	case GRAPH_MODE_RAW:
		accGraph->addData(0, ld.a[0]);
		accGraph->addData(1, ld.a[1]);
		accGraph->addData(2, ld.a[2]);
		gyroGraph->addData(0, ld.g[0]);
		gyroGraph->addData(1, ld.g[1]);
		gyroGraph->addData(2, ld.g[2]);
		magGraph->addData(0, ld.b[0]);
		magGraph->addData(1, ld.b[1]);
		magGraph->addData(2, ld.b[2]);
	break; 
	
	case GRAPH_MODE_ORIENTATION:
		quaternionGraph->addData(0, ld.q[0]);
		quaternionGraph->addData(1, ld.q[1]);
		quaternionGraph->addData(2, ld.q[2]);
		quaternionGraph->addData(3, ld.q[3]);	
		angleGraph->addData(0, ld.r[0]);
		angleGraph->addData(1, ld.r[1]);
		angleGraph->addData(2, ld.r[2]);
		linAccGraph->addData(0, ld.linAcc[0]);
		linAccGraph->addData(1, ld.linAcc[1]);
		linAccGraph->addData(2, ld.linAcc[2]);
	break;

	case GRAPH_MODE_PRESSURE:		
		pressureGraph->addData(0, ld.pressure);
		altitudeGraph->addData(0, ld.altitude);
		temperatureGraph->addData(0, ld.temperature);
	break;
	
	case GRAPH_MODE_HEAVEMOTION:
		heaveMotionGraph->addData(0, ld.hm.yHeave);
	break;
	}
}

void GraphWindow::clearGraphs(void)
{
	accGraph->clearData();
	gyroGraph->clearData();
	magGraph->clearData();
	quaternionGraph->clearData();
	angleGraph->clearData();
	pressureGraph->clearData();
	linAccGraph->clearData();
	altitudeGraph->clearData();
	temperatureGraph->clearData();
	heaveMotionGraph->clearData();
}
	
void GraphWindow::setActiveLpms(int openMatId)
{
	activeOpenMatId = openMatId;
}