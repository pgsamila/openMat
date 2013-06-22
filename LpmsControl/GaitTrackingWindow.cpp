#include "GaitTrackingWindow.h"
	
GaitTrackingWindow::GaitTrackingWindow(QWidget *parent)
{	
	QHBoxLayout *h0 = new QHBoxLayout();
	setLayout(h0);
	
	QVBoxLayout *vl0 = new QVBoxLayout();	
	QVBoxLayout *vl1 = new QVBoxLayout();	
	QVBoxLayout *vl2 = new QVBoxLayout();	
	QVBoxLayout *vl3 = new QVBoxLayout();	
	QVBoxLayout *rl = new QVBoxLayout();
	
	g0 = new VerticalBarGraph(1);
	vl0->addWidget(g0);
	vl0->addWidget(new QLabel("Step height"));
	vl0->setStretch(0, 10);
	vl0->setStretch(1, 0);
	
	g1 = new VerticalBarGraph(1);
	vl1->addWidget(g1);
	vl1->addWidget(new QLabel("Step length"));
	vl1->setStretch(0, 10);
	vl1->setStretch(1, 0);
	
	d0 = new RepDisplay();
	vl2->addWidget(d0);
	vl2->addWidget(new QLabel("Step height (rel. units)"));
	vl2->setStretch(0, 10);
	vl2->setStretch(1, 0);
	
	d1 = new RepDisplay();
	vl2->addWidget(d1);
	vl2->addWidget(new QLabel("Step length (rel. units)"));
	vl2->setStretch(2, 10);
	vl2->setStretch(3, 0);

	d2 = new RepDisplay();
	vl3->addWidget(d2);
	vl3->addWidget(new QLabel("Step frequency (steps/min)"));
	vl3->setStretch(0, 10);
	vl3->setStretch(1, 0);
	
	d3 = new RepDisplay();
	vl3->addWidget(d3);
	vl3->addWidget(new QLabel("Gait velocity (rel. units/s)"));
	vl3->setStretch(2, 10);
	vl3->setStretch(3, 0);
	
	d4 = new RepDisplay();
	vl3->addWidget(d4);
	vl3->addWidget(new QLabel("Symmetry (unitless)"));
	vl3->setStretch(4, 10);
	vl3->setStretch(5, 0);
	
	h0->addLayout(vl0);
	h0->addLayout(vl1);
	h0->addLayout(vl2);
	h0->addLayout(vl3);
	
	g0->show();
	g0->setRange(-1.0f, 1.0f);
	
	g1->show();
	g1->setRange(-1.0f, 1.0f);
}

GaitTrackingWindow::~GaitTrackingWindow()
{
}

void GaitTrackingWindow::update(ImuData d)
{
	if (d.gm.zDirection == 0) {
		g0->setColor(0);
	} else {
		g0->setColor(1);
	}
	
	if (d.gm.yDirection == 0) {
		g1->setColor(0);
	} else {
		g1->setColor(1);
	}
	
	g0->setValue(d.gm.zGait);
	g1->setValue(d.gm.yGait);
	d0->setValue(d.gm.zAmplitude);
	d1->setValue(d.gm.yAmplitude);
	d2->setValue(d.gm.frequency);
	d3->setValue(d.gm.velocity);
	d4->setValue(d.gm.symmetry);
}