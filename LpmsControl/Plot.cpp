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

#include "Plot.h"

#include <iostream>

#include <QGridLayout>

Plot::Plot(string title, string xAxis, string yAxis, 
	string aName, string bName, string cName, string dName, 
	int color, int maxData, int nCurves, 
	float yMax, float yMin, 
	QwtLegend *legend,
	QWidget *parent) 
	: QwtPlot(parent),
	yMax(yMax),
	yMin(yMin)
{
	/* (void) new QwtPlotPanner(canvas());
	(void) new QwtPlotMagnifier(canvas());
	setMinimumSize(100, 150); */
	
	this->nCurves = nCurves;
	
	if (maxData < MAX_DATA) {
		this->maxData = maxData;
	} else {
		this->maxData = MAX_DATA;
	}

    setAutoFillBackground(true);
    setPalette(QPalette(QColor(255, 255, 255)));

	QwtDynGridLayout *dgl = qobject_cast<QwtDynGridLayout *>(legend->contentsWidget()->layout());
	dgl->setMaxCols(1);
	legend->setMinimumWidth(125);
	legend->contentsWidget()->layout()->setAlignment(Qt::AlignLeft);
	
	insertLegend(legend, QwtPlot::ExternalLegend);

	QwtText qwtTitle(title.c_str());

	QFont f;
	f.setUnderline(true);
	f.setPointSize(9);
	f.setBold(false);	

	qwtTitle.setFont(f);
	// setTitle(qwtTitle);
	
	QwtText t0(QString(xAxis.c_str()));
	QwtText t1(QString(yAxis.c_str()));

	f.setUnderline(false);	
	t0.setFont(f);	
	t1.setFont(f);	

	setAxisFont(xBottom, f);
    setAxisFont(yLeft, f);	
	
    setAxisTitle(xBottom, t0);
    setAxisScale(xBottom, 0.0, (float) maxData);
    setAxisTitle(yLeft, t1);

    setAxisScale(yLeft, yMax, yMin);	
	setAxisMaxMajor(yLeft, 5);
	
	/* setAxisAutoScale(yLeft);
    setAxisScale(yLeft, -5000, 5000);
	QwtPlot::setFrameStyle(QFrame::NoFrame);
  	QwtPlot::setLineWidth(0);
  	QwtPlot::setCanvasLineWidth(2); */
	
  	QwtPlot::setAutoReplot();

    // panning with the left mouse button
    (void) new QwtPlotPanner(canvas());

    // zoom in/out with the wheel
    (void) new QwtPlotMagnifier(canvas());	
	
    canvas()->setLineWidth(1);
	// canvas()->setFrameStyle(QFrame::NoFrame);
    canvas()->setFrameStyle(QFrame::Box | QFrame::Plain);

    QPalette canvasPalette(Qt::white);
    canvasPalette.setColor(QPalette::Foreground, QColor(0, 0, 0));
    canvas()->setPalette(canvasPalette); 
	
	QwtScaleWidget *scaleWidget = axisWidget(QwtPlot::yLeft);
	scaleWidget->scaleDraw()->setMinimumExtent(50);	
	
	scaleWidget = axisWidget(QwtPlot::yRight);
	scaleWidget->scaleDraw()->setMinimumExtent(200);	

	QwtPlotGrid *grid = new QwtPlotGrid;
    grid->enableX(true);
    grid->enableY(true);
    grid->setMajPen(QPen(Qt::gray, 0, Qt::DotLine));
    grid->setMinPen(QPen(Qt::gray, 0, Qt::DotLine));
    grid->attach(this);
	
	/* QPalette p(palette());
    canvasPalette.setColor(backgroundRole(), Qt::white);
    canvas()->setPalette(p); */
	
	curveName[0] = aName; // std::string("X-Axis");
	curveName[1] = bName; // std::string("Y-Axis");
	curveName[2] = cName; // std::string("Z-Axis");
	curveName[3] = dName; // std::string("W-Axis");	

	// QPen redPen(QColor(255, 153, 51));
	QPen redPen(Qt::red);
	redPen.setWidth(2);
	// QPen greenPen(QColor(0, 204, 153));
	QPen greenPen(Qt::green);
	greenPen.setWidth(2);
	// QPen bluePen(QColor(102, 153, 255)); 
	QPen bluePen(Qt::blue);
	bluePen.setWidth(2);
	// QPen magentaPen(QColor(255, 102, 255));
	QPen magentaPen(Qt::magenta);
	magentaPen.setWidth(2);
	
	for (int i=0; i<nCurves; i++)
	{
		Curve c;
		
		c.qwtCurve = new QwtPlotCurve(curveName[i].c_str());
		c.qwtCurve->setTitle(QString(curveName[i].c_str())+QString(" = +00.00"));
		c.nData = 0;
		c.xPos = 0;
		c.first = true;

		switch (i) {
		case 2:
			c.qwtCurve->setPen(redPen);
			break;

		case 1:
			c.qwtCurve->setPen(greenPen);
			break;
			
		case 0:
			c.qwtCurve->setPen(bluePen);
			break;

		default:
			c.qwtCurve->setPen(magentaPen);
			break;
		}
		
		c.qwtCurve->attach(this);
		c.qwtCurve->setRawSamples(c.xData, c.yData, c.nData); 
		
		curves.push_back(c);
	}

	setXMarker("");
	
	clearData();
	
	textRedraw[0] = textRedraw[1] = textRedraw[2] = textRedraw[3] = 0;
}

QSize Plot::minimumSizeHint() const
{
	return QSize(150, 100);
}

void Plot::addData(int i, float y)
{	
	if (i < nCurves) {
		if (curves[i].xPos /*curves[i].nData*/ > maxData-1) {
			// curves[i].nData = 0;
			curves[i].xPos = 0;
			curves[i].nData = maxData;
			curves[i].first = false;
			// clearXMarker();
		}

		if (curves[i].first == true) curves[i].nData = curves[i].xPos+1;
	
		curves[i].xData[curves[i].xPos /*curves[i].nData*/] = (float) curves[i].xPos; // curves[i].nData;
		curves[i].yData[curves[i].xPos /*curves[i].nData*/] = (float) y;
				
		setData(i);

		if (markerX.size() > 0) markerX[0]->setXValue((float) curves[i].xPos);

		++curves[i].xPos;
		
		if (textRedraw[i] > 1) {
			textRedraw[i] = 0;

			QString text;
			text.sprintf(" = %+06.2f", y);
			curves[i].qwtCurve->setTitle(QString(curveName[i].c_str()) + QString(" = ") + text);
		} else {
			++textRedraw[i];
		}
	}
}

void Plot::setXMarker(string text)
{	
	QwtPlotMarker* m;
	
    m = new QwtPlotMarker();
    // m->setLabel(QString::fromStdString(text));
    m->setLabelAlignment(Qt::AlignRight | Qt::AlignTop);
	// m->setLabelOrientation(Qt::Horizontal);
    m->setLineStyle(QwtPlotMarker::VLine);
    m->setLinePen(QPen(Qt::gray, 0, Qt::SolidLine));
    m->setXValue(0); // curves[0].nData);

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
	if (i < nCurves) {	
		curves[i].qwtCurve->setRawSamples(curves[i].xData, curves[i].yData, curves[i].nData); 
	}
}

void Plot::clearData(void)
{
	for (int i=0; i<nCurves; i++)
	{
		curves[i].nData = 0; // maxData;
		curves[i].first = true;

		for (int j=0; j<maxData; j++) {
			curves[i].xData[j] = 0.0f;
			curves[i].yData[j] = 0.0f;
		}

		curves[i].nData = 0;
		curves[i].xPos = 0;
	
		curves[i].qwtCurve->setRawSamples(curves[i].xData, curves[i].yData, curves[i].nData); 	
	}
	
	markerX[0]->setXValue(0);
	
	setAxisScale(xBottom, 0.0, (float) maxData);
    setAxisScale(yLeft, yMax, yMin);
	
	// clearXMarker();
}
