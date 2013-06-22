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

#ifndef CUBE_WINDOW_CONTAINER
#define CUBE_WINDOW_CONTAINER

#include <QWidget>
#include <QGridLayout>
#include <QComboBox>
#include <QLabel>
#include <QHBoxLayout>

#include "ImuData.h"
#include "ThreeDWindow.h"

#define CUBE_VIEW_MODE_1 1
#define CUBE_VIEW_MODE_2 2
#define CUBE_VIEW_MODE_4 3

class CubeWindowContainer : public QWidget 
{
Q_OBJECT

public:
	QGridLayout* fourLayout;
	vector<ThreeDWindow *> cubeWindows;
	vector<QComboBox *> comboBoxes;
	bool created;
	int mode;

	CubeWindowContainer(int m, QWidget *parent = 0) :
		QWidget(parent)
	{
		created = false;
		setViewMode(m);
	}
	
	~CubeWindowContainer()
	{
		for (unsigned i=0; i<cubeWindows.size(); ++i) {		
			delete cubeWindows[i];
		}
	}
	
	void setViewMode(int m) 
	{
		mode = m;

		fourLayout = new QGridLayout();
		
		switch (m) {
		case CUBE_VIEW_MODE_1:
			cubeWindows.push_back(new ThreeDWindow());
			fourLayout->addWidget(cubeWindows[0], 0, 0);
		break;
		
		case CUBE_VIEW_MODE_2:
			fourLayout->setHorizontalSpacing(25);
			fourLayout->setVerticalSpacing(10);			

			for (int i=0; i<2; ++i) {
				comboBoxes.push_back(new QComboBox());
				for (int j=1; j<33; j++) comboBoxes[i]->addItem(QString("%1").arg(j));
				comboBoxes[i]->setCurrentIndex(i);
				
				QHBoxLayout *h = new QHBoxLayout();
				h->addWidget(new QLabel("IMU ID: "));
				h->addWidget(comboBoxes[i]);
				h->addStretch();
				
				cubeWindows.push_back(new ThreeDWindow());

				QVBoxLayout *v = new QVBoxLayout();			
				v->addStretch();
				v->addLayout(h);
				v->addSpacing(5);
				v->addWidget(cubeWindows[i]);
				v->addStretch();
				v->setStretch(0, 1);
				v->setStretch(1, 1);
				v->setStretch(2, 1);
				v->setStretch(3, 5);
				v->setStretch(4, 1);

				fourLayout->addLayout(v, 0, i);	
			}	
		break;

		case CUBE_VIEW_MODE_4:
			fourLayout->setHorizontalSpacing(10); 
			fourLayout->setVerticalSpacing(10); 		
			for (int y=0; y<2; ++y) {
				for (int x=0; x<2; ++x) {
					comboBoxes.push_back(new QComboBox());
					for (int j=1; j<33; j++) comboBoxes[y*2+x]->addItem(QString("%1").arg(j));
					comboBoxes[y*2+x]->setCurrentIndex(y*2+x);
					
					QHBoxLayout *h = new QHBoxLayout();
					h->addWidget(new QLabel("IMU ID: "));
					h->addWidget(comboBoxes[y*2+x]);
					h->addStretch();
					
					cubeWindows.push_back(new ThreeDWindow());

					QVBoxLayout *v = new QVBoxLayout();			
					v->addLayout(h);
					v->addSpacing(5);
					v->addWidget(cubeWindows[y*2+x]);

					fourLayout->addLayout(v, y, x);
				}
			}
		break;
		}
		
		setLayout(fourLayout);
		for (unsigned i=0; i<cubeWindows.size(); ++i) cubeWindows[i]->show();
		
		created = true;
	}
	
	void updateData(ImuData d) 
	{
		if (getMode() == CUBE_VIEW_MODE_1) {
			cubeWindows[0]->updateQuaternion(d);
		} else {		
			for (unsigned i=0; i<cubeWindows.size(); ++i) {
				if (d.openMatId == (comboBoxes[i]->currentIndex()+1)) {
					cubeWindows[i]->updateQuaternion(d);
				}
			}
		}
	}
	
	int getMode(void) 
	{
		return mode;
	}
};

class CubeWindowSelector : public QWidget
{
Q_OBJECT

public:
	QVBoxLayout* v;
	CubeWindowContainer *c0;
	CubeWindowContainer *c1;
	CubeWindowContainer *c2;
	CubeWindowContainer *sc;
	int mode;
	
	int getMode(void) 
	{
		return mode;
	}	
	
	CubeWindowContainer* getSelectedCube(void)
	{
		return sc;
	}
	
	CubeWindowSelector(void)
	{
		c0 = new CubeWindowContainer(CUBE_VIEW_MODE_1);
		c1 = new CubeWindowContainer(CUBE_VIEW_MODE_2);
		c2 = new CubeWindowContainer(CUBE_VIEW_MODE_4);
		
		v = new QVBoxLayout();
		v->addWidget(c0);
		v->addWidget(c1);
		v->addWidget(c2);
		
		setLayout(v);
		
		c0->show();
		c1->hide();
		c2->hide();
		
		sc = c0;
		
		mode = CUBE_VIEW_MODE_1;
	}
	
	void selectCube(int i)
	{
		mode = i;
	
		switch (i) {
		case CUBE_VIEW_MODE_1:
			c1->hide();
			c2->hide();	
			c0->show();
			sc = c0;
		break;
		
		case CUBE_VIEW_MODE_2:
			c0->hide();
			c2->hide();		
			c1->show();
			sc = c1;
		break;
		
		case CUBE_VIEW_MODE_4:
			c0->hide();
			c1->hide();
			c2->show();		
			sc = c2;
		break;
		}
	}
};

#endif