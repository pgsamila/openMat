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

#include "Plot.h"

Plot::Plot(string title, string xAxis, string yAxis, int color, int maxData, int nCurves, double yMax, double yMin, QWidget *parent) 
	: QwtPlot(parent)
{	
	this->nCurves = nCurves;
	
	if (maxData < MAX_DATA) {
		this->maxData = maxData;
	} else {
		this->maxData = MAX_DATA;
	}

    setAutoFillBackground(true);
    setPalette(QPalette(QColor(255, 255, 255)));

    setTitle(title.c_str());
	QFont f;
	f.setUnderline(true);
	f.setPointSize(10);
	titleLabel()->setFont(f);
	
	f.setBold(false);
	QwtText t(QString(xAxis.c_str()));
    setAxisTitle(xBottom, t);
    setAxisScale(xBottom, 0.0, (double) maxData);
	t.setFont(f);

    setAxisScale(yLeft, yMax, yMin);	
	setAxisMaxMajor(yLeft, 5);
	setAxisMaxMajor(xBottom, 5);	
	
  	QwtPlot::setAutoReplot();

    canvas()->setLineWidth(1);
    canvas()->setFrameStyle(QFrame::Box | QFrame::Plain);

    QPalette canvasPalette(Qt::white);
    canvasPalette.setColor(QPalette::Foreground, QColor(0, 0, 0));
    canvas()->setPalette(canvasPalette); 

	QwtPlotGrid *grid = new QwtPlotGrid;
    grid->enableX(true);
    grid->enableY(false);
    grid->setMajPen(QPen(Qt::gray, 0, Qt::DotLine));
    grid->setMinPen(QPen(Qt::gray, 0 , Qt::DotLine));
    grid->attach(this);

	for (int i=0; i<nCurves; i++) {
		Curve c;
		int d;
		
		c.qwtCurve = new QwtPlotCurve();
		c.nData = 0;

		if (color > 0) {
			d = color - 1;
		} else {
			d = i;
		}
		
		switch (d)
		{
		case 0:
			c.qwtCurve->setPen(QPen(Qt::red));
			break;

		case 1:
			c.qwtCurve->setPen(QPen(Qt::green));
			break;
			
		case 2:
			c.qwtCurve->setPen(QPen(Qt::blue));
			break;

		default:
			c.qwtCurve->setPen(QPen(Qt::magenta));
			break;
		}
		c.qwtCurve->attach(this);
		c.qwtCurve->setRawSamples(c.xData, c.yData, c.nData); 
		curves.push_back(c);
	}
	clearData();
}

void Plot::addData(int i, double y)
{	
	if (i<nCurves) {
		if (curves[i].nData > maxData-1) {
			curves[i].nData = 0;
			
			clearXMarker();
		}
		curves[i].xData[curves[i].nData] = (double) curves[i].nData;
		curves[i].yData[curves[i].nData] = (double) y;
		setData(i);
		++curves[i].nData;
	}
}

void Plot::setXMarker(string text)
{	
	QwtPlotMarker* m;
	
    m = new QwtPlotMarker();
    m->setLabel(QString::fromStdString(text));
    m->setLabelAlignment(Qt::AlignRight | Qt::AlignTop);
	m->setLabelOrientation(Qt::Horizontal);
    m->setLineStyle(QwtPlotMarker::VLine);
    m->setLinePen(QPen(Qt::gray, 0, Qt::SolidLine));
    m->setXValue(curves[0].nData);
    m->attach(this);
	markerX.push_back(m);
}

void Plot::clearXMarker(void)
{
	for (unsigned int i=0; (int) i<markerX.size(); i++) {
		markerX[i]->detach();
		delete markerX[i];
	}
	
	markerX.clear();
}

void Plot::setData(int i)
{
	if (i<nCurves) {	
		curves[i].qwtCurve->setRawSamples(curves[i].xData, curves[i].yData, curves[i].nData); 
	}
}

void Plot::clearData(void)
{
	for (int i=0; i<nCurves; i++) {
		curves[i].nData = 0;
		curves[i].qwtCurve->setRawSamples(curves[i].xData, curves[i].yData, curves[i].nData); 	
	}
	
	clearXMarker();
}