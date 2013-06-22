#include <iostream>
#include <string>

#include <QApplication>
#include <QThread>
#include <QSplashScreen>
#include <QWaitCondition>

#include "MainWindow.h"
#include "Analysis.h"
#include "HmmAnalysis.h"

#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{		
	qRegisterMetaType<LpmsData>("LpmsData");
	qRegisterMetaType<LpmsSpectrumData>("LpmsSpectrumData");
	
	QApplication a(argc, argv);
	a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit())); 

	QPixmap pixmap("LpmsControl.png");
	QSplashScreen splash(pixmap);
	splash.show();
	
	cout << "[LPMS] Starting up LpmsControl software." << endl;
		
	MainWindow mw;
	mw.show();
	
	splash.finish(&mw);
	
	a.exec();

	return 0;
}