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

#ifndef GRAPH_WINDOW
#define GRAPH_WINDOW

#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QComboBox>
#include <QGroupBox>
#include <QPalette>

#include "Plot.h"
#include "ImuData.h"

#define GRAPH_MODE_RAW 0
#define GRAPH_MODE_ORIENTATION 1
#define GRAPH_MODE_PRESSURE 2
#define GRAPH_MODE_HEAVEMOTION 3

/* Contains the sensor data plots. */
class GraphWindow : public QWidget
{
Q_OBJECT
	
public:
	GraphWindow(QWidget *parent = 0);
	void clearGraphs(void);
	void setActiveLpms(int openMatId);
	void setMode(int m);
	
public slots:
	void plotDataSet(ImuData ld);
	
public:
	Plot *accGraph;	
    Plot *gyroGraph;	
    Plot *magGraph;	
    Plot *angleGraph;	    
	Plot *quaternionGraph;
	Plot *pressureGraph;
	Plot *altitudeGraph;
	Plot *temperatureGraph;
	Plot *linAccGraph;
	Plot *heaveMotionGraph;
	
	QwtLegend* accLegend;
	QwtLegend* gyroLegend;
	QwtLegend* magLegend;
	QwtLegend* quaternionLegend;
	QwtLegend* angleLegend;
	QwtLegend* pressureLegend;
	QwtLegend* altitudeLegend;
	QwtLegend* temperatureLegend;
	QwtLegend* linAccLegend;	
	QwtLegend* heaveMotionLegend;
	
	int activeOpenMatId;
	int mode;
};

#endif