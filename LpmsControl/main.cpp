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

/*! \mainpage LpmsControl Documentation	
	\section intro_sec Introduction
	LpmsControl works as a base-program for OpenMAT to collect data from IMU sensors.
	At the moment it contains drivers to work with the LP-Research (LPMS) IMUs. In 
	later development stages support is to be extended to 3rd party IMUs. The program
	manages multiple sensors at the same time, analyzes the raw data and converts it
	into orientation data using the LP-Filter. It provides several visualization and
	options and allows to save the data sampled from the sensors. All acquired 
	information can be sent to other applications such as the Human Body Simulator and
	be processed further.
*/

#include <iostream>
#include <string>

#include <QApplication>
#include <QThread>
#include <QSplashScreen>
#include <QWaitCondition>
#include <QFont>

#include "MainWindow.h"

#include <iostream>

using namespace std;
#ifdef _WIN32
	#ifndef SHOW_CONSOLE
		int __stdcall WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, char*, int nShowCmd)
	#else
		int main(int argc, char *argv[])
	#endif
#endif 

#ifdef __GNUC__
	int main(int argc, char *argv[])
#endif
{		
	qRegisterMetaType<ImuData>("ImuData");
	
#ifdef _WIN32
	QApplication a(__argc, __argv);
#endif

#ifdef __GNUC__
	QApplication a(argc, argv);
#endif

	a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit())); 

    QFile file("LpmsStyles.qss");
    file.open(QFile::ReadOnly);
    QString styleSheet = QLatin1String(file.readAll());
	a.setStyleSheet(styleSheet);
	
	/* QFont font("Arial", 10);
	a.setFont(font); */
	
	QPixmap pixmap("LpSplash.png");
	QSplashScreen splash(pixmap);
	splash.show();
		
	MainWindow mw;
	mw.show();
		
	splash.finish(&mw);
	
	a.exec();

	return 0;
}
