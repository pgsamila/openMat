#ifndef GAIT_TRACKING_WINDOW
#define GAIT_TRACKING_WINDOW

#include <QWidget>

#include <QPushButton>
#include <QHBoxLayout>
#include <QLabel>

#include "VerticalBarGraph.h"
#include "ImuData.h"

class GaitTrackingWindow : public QWidget
{
Q_OBJECT
public:
	GaitTrackingWindow(QWidget *parent = 0);
	~GaitTrackingWindow();
	
public slots:
	void update(ImuData d);
	
public:
	VerticalBarGraph *g0;
	VerticalBarGraph *g1;
	RepDisplay *d0;
	RepDisplay *d1;
	RepDisplay *d2;
	RepDisplay *d3;
	RepDisplay *d4;
};

#endif