/***********************************************************************
** (c) LP-RESEARCH Inc.
** All rights reserved
** Contact: info@lp-research.com
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

#ifndef LP_MAIN_WINDOW
#define LP_MAIN_WINDOW

#ifdef _WIN32
	#include "windows.h"
#endif

#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QComboBox>
#include <QGroupBox>
#include <QListWidget>
#include <QMessageBox>
#include <QTreeWidget>
#include <QLineEdit>
#include <QGridLayout>
#include <QTreeWidgetItem>
#include <QTreeWidget>
#include <QButtonGroup>
#include <QMainWindow>
#include <QWizard>
#include <QWizardPage>
#include <QSpacerItem>
#include <QSizePolicy>

#include "Plot.h"
#include "GraphWindow.h"
#include "ThreeDWindow.h"
#include "SensorGuiContainer.h"
#include "LpmsSensorManagerI.h"
#include "DiscoveryTree.h"
#include "LpmsSensorI.h"
#include "LpmsDefinitions.h"
#include "FieldMapContainer.h"
#include "RescanDialog.h"
#include "CubeWindowContainer.h"
#include "MicroMeasure.h"
#include "GaitTrackingWindow.h"
#include "MotionBuilderCommunication.h"
#include "PlayControl.h"

#include <string>
#include <iostream>
#include <vector>
#include <list>

#include <boost/shared_ptr.hpp>
using namespace boost;

#define CALIBRATION_FILE "LpmsControlConfiguration.xml"

#define MODE_GRAPH_WIN 0
#define MODE_THREED_WIN 1
#define MODE_FIELDMAP_WIN 2
#define MODE_GAIT_TRACKING_WIN 3

#define LPMS_CONTROL_VERSION "1.3.5 (4EEEE4B)"

#define GRAPH_UPDATE_PERIOD 25000

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();

public slots:	
	QWidget *createDeviceList(void);
	QGroupBox *createGraphs(void);
	void createMenuAndToolbar(void);
	void closeEvent(QCloseEvent *event);

	void selectGraphWindow(void);
	void selectGraph2Window(void);
	void selectGraph3Window(void);
	void selectHeaveMotionWindow(void);
	void selectThreeDWindow(void);
	void selectFieldMapWindow(void);
	void selectCubeMode1(void);
	void selectCubeMode2(void);
	void selectCubeMode4(void);
	bool exitWindow(void);

	void timerUpdate(void);

	void updateCanBaudrate(int i);
	void updateRs232Baudrate(int i);
	void updateCurrentLpms(QTreeWidgetItem *current = 0, QTreeWidgetItem *previous = 0);
	void checkOptionalFeatures(LpmsSensorI* sensor);

	void openSensor(void);
	void closeSensor(void);
	void toggleSelfTest(void);
	void startMeasurement(void);
	void stopMeasurement(void);

	void setOffset(void);
	void resetOffset(void);
	void resetHeading(void);
	void armTimestampReset(void);
	void recalibrate(void);

	void uploadFirmware(void);
	void cancelUpload(void);
	void uploadTimerUpdate(void);
	void uploadIap(void);
	void recordData(void);
	void browseRecordFile(void);

	void saveCalibration(void);
	void addRemoveDevices(void);
	void saveCalibrationData(void);
	void loadCalibrationData(void);

	void updateLpmsFps(int v, int lpmsId);
	void measureLatency(void);
	void getVersionInfo(void);
	void resetToFactory(void);

	void startWaitBar(int t);
	void calTimerUpdate(void);

	void magAutoMisalignmentCal(void);
	void magMisalignmentCal(void);
	void magMaNewPage(int i);
	void magMaFinished(int i);
	QWizardPage *magMaOrientationPage(const char* ts, const char* es);
	QWizardPage *magMaFinishedPage(void);

	void gyrMisalignmentCal(void);
	void gyrMaNewPage(int i);
	void gyrMaFinished(int i);
	QWizardPage *gyrMaOrientationPage(const char* ts, const char* es);
	QWizardPage *gyrMaFinishedPage(void);

	QWizardPage *magEllipsoidCalPage(int pageNumber);
	void magEllipsoidCalNewPage(int i);
	void calibrateMag(void);

	QWizardPage *magPlanarCalPage(int pageNumber);
	void magPlanarCalNewPage(int i);
	void calibratePlanarMag(void);

	QWizardPage *maOrientationPage(const char* ts, const char* es);
	QWizardPage *maFinishedPage(void);
	void misalignmentCal(void);
	void maNewPage(int i);
	void maFinished(int i);

	void updateMagneticFieldMap(void);
	void getFieldMap(void);
	void startReplay(void);
	void stopReplay(void);
	void browsePlaybackFile(void);
	void loadObjFile(void);
	
private:
	QList<QTreeWidgetItem*> lpmsTreeItems;
	QTreeWidget *lpmsTree;	
	GraphWindow *graphWindow;
	CubeWindowSelector *cubeWindowContainer;
	FieldMapContainer *fieldMapWindow;
	SensorGuiContainer *currentLpms;
	list<SensorGuiContainer *> lpmsList;
	LpmsSensorManagerI *sm;
	QLabel *imuFpsLabel;
	QLabel *bluetoothStatusLabel;
	QLabel *lpmsStatusLabel;
	QLabel *xAccLabel;
	QLabel *yAccLabel;
	QLabel *zAccLabel;
	QLabel *xGyroLabel;
	QLabel *yGyroLabel;
	QLabel *zGyroLabel;
	QLabel *xMagLabel;
	QLabel *yMagLabel;
	QLabel *zMagLabel;
	QLabel *xAngleLabel;
	QLabel *yAngleLabel;
	QLabel *zAngleLabel;
	QLabel *xQuatLabel;
	QLabel *yQuatLabel;
	QLabel *zQuatLabel;
	QLabel *wQuatLabel;
	QLabel *frameCountLabel; 
	QLabel *activeLpmsLabel; 
	QLabel *magRangeLabel;
	QLabel *pressureLabel;
	QLabel *xLinAccLabel;
	QLabel *yLinAccLabel;
	QLabel *zLinAccLabel;
	QLineEdit *deviceAddressEdit;
	QLineEdit *ftdiDeviceEdit;
	QPushButton *recordDataButton;
	QPushButton *startButton;
	QPushButton *connectButton;
	QPushButton *calibrateMagButton;
	QPushButton *startServerButton;	
	QPushButton *graphWindowButton;
	QPushButton *graphWindow2Button;
	QPushButton *graphWindow3Button;
	QPushButton *threeDWindowButton;
	QPushButton *fieldMapWindowButton;
	bool isRunning;
	bool isConnecting;	
	bool calibratingMag;
	QSlider *accGainSl;
	QSlider *magGainSl;
	QSlider *accCovarSl;
	QSlider *magCovarSl;	
	QLineEdit *csvFileEdit;
	QRadioButton* gyrOnlyBtn;
    QRadioButton* gyrAccBtn;
    QRadioButton* accMagBtn;
    QRadioButton* gyrAccMagNsBtn;
    QRadioButton* gyrAccMagSwBtn;
	QRadioButton* gTEnableBtn;
	QRadioButton* gTDisableBtn;
	QComboBox* deviceTypeSelector;
	QComboBox *addressSelector;
	QComboBox *recordingRateCombo;
	QAction* startAction;
	QAction* saveAction;
	QAction* selfTestAction;
	QAction* cubeMode1Action;
	QAction* cubeMode2Action;
	QAction* cubeMode4Action;
	int textUpdateCounter;
	QTimer* uploadTimer;
	QProgressDialog* uploadProgress;
	QWizard* maWizard;
	bool maCalibrationFinished;
	int calMaxTime;
	int calTime;
	QProgressDialog *calProgress;
	QTimer *calTimer;
	bool maIsCalibrating;
	RescanDialog* rescanD;
	int mode;
	QGridLayout *cubeLayout;
	QVBoxLayout* graphLayout;
	QComboBox *comboDeviceList;
	string globalRecordFile;
	QLineEdit *recordFileEdit;
	QComboBox *targetCombo;
	QComboBox *resetMethodCombo;
	bool recordFileSet;
	MicroMeasure mm;
	QComboBox *canBaudrateList;
	QComboBox *rs232BaudrateList;
	QToolBar *toolbar;
	QMenu* viewMenu;
	bool heaveMotionEnabled;
	MotionPlayer *rePlayer;
	QAction *replayAction;
	string globalPlaybackFile;
	QLineEdit *playbackFileEdit;
	bool playbackFileSet;
	bool isMagEllipsoidCalOn;
	bool isMagPlanarCalOn;
	QWizard* magEllipsoidCalWizard;
	QWizard* magPlanarCalWizard;
	//MotionBuilderCommunication mbcom;
};

#endif
