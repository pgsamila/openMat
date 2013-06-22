/****************************************************************************
**
** Copyright (C) 2011 LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** OpenMAT is free software: you can redistribute it and/or modify it under 
** the terms of the GNU General Public License as published by the Free 
** Software Foundation, either version 3 of the License, or (at your option) 
** any later version.
** 
** OpenMAT is distributed in the hope that it will be useful, but WITHOUT 
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
** FITNESS FOR A PARTICULAR PURPOSE. See the GNU \ General Public License 
** for more details.
** 
** You should have received a copy of the GNU General Public License along 
** with the OpenMAT library. If not, see <http://www.gnu.org/licenses/>.
**
****************************************************************************/

#include "StdRedirector.h"

QMutex coutMutex;

void outcallback(const char* ptr, std::streamsize count, void* bufferString)
{
	string *b = (string *) bufferString;	
	string t;
	
	for (int i=0; i < count; i++)
	{
		if (ptr[i] == '\n')
		{
			t = t + "\n";
		} else {
			t = t + ptr[i];
		}
	}		
	
	coutMutex.lock();
	*b = *b + t;
	coutMutex.unlock();
}
	
void ConsoleWindow::updateTimer(void)
{	
	coutMutex.lock();
	if (bufferString.size() > 0)
	{
		consoleBox->insertPlainText(QString(bufferString.c_str()));
		bufferString.clear();
		
		QScrollBar *sb = consoleBox->verticalScrollBar();
		sb->setValue(sb->maximum());		
	}
	coutMutex.unlock();
}

ConsoleWindow::ConsoleWindow(QWidget *parent) : QWidget(parent)
{
	consoleBox = new QTextEdit(this);
	consoleBox->setReadOnly(true);
	
	stdRedirector = new StdRedirector<>(std::cout, outcallback, &bufferString);
	
	QVBoxLayout *vb = new QVBoxLayout();
	vb->addWidget(consoleBox);
	vb->setMargin(0);
	vb->setSpacing(0);	
	
	setLayout(vb);
	
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(updateTimer()));
	timer->start(100);
}

ConsoleWindow::~ConsoleWindow()
{
	delete stdRedirector;
}