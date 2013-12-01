/***********************************************************************
** Copyright (C) 2012 LP-Research
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

#ifdef USE_ZEROC_ICE
	#include "IceStormCommunication.h"
#endif

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
//birdy
#include "MotionBuilderCommunication.h"

#include <string>
#include <iostream>
#include <vector>
#include <list>
using namespace std;

#include <boost/shared_ptr.hpp>
using namespace boost;

#define CALIBRATION_FILE "LpmsControlConfiguration.xml"

#define MODE_GRAPH_WIN 0
#define MODE_THREED_WIN 1
#define MODE_FIELDMAP_WIN 2
#define MODE_GAIT_TRACKING_WIN 3

#define USE_HEAVEMOTION

#define LPMS_CONTROL_VERSION "1.2.5"

/* LPMS Control main window. */
class MainWindow : public QMainWindow
{
Q_OBJECT
	
public:
	/* Initializes main application window. */
	MainWindow(QWidget *parent = 0);
	
	/* Destroys the MainWindow. */
	~MainWindow();
	
	/* Creates graphs and 3D visualization. */
	QGroupBox *createGraphs(void);

	/* Creates LPMS device list. */
	QWidget *createDeviceList(void);
	
	/* Creates accelerometer misalignment orientation wizard page. */
	QWizardPage *maOrientationPage(const char* ts, const char* es);

	/* Creates gyroscope misalignment orientation wizard page. */	
	QWizardPage *gyrMaOrientationPage(const char* ts, const char* es);

public slots:	
	/* Displays graph window. */
	void selectGraphWindow(void);
	
	/* Displays 2nd graph window. */	
	void selectGraph2Window(void);
	
	/* Displays 3rd graph window. */	
	void selectGraph3Window(void);

	/* Displays 3D visualization. */
	void selectThreeDWindow(void);
	
	/* Closes application. */
	bool exitWindow(void);

	/* Creates new sensor connection. */
	void openSensor(void);
	
	/* Closes sensor connection. */
	void closeSensor(void);

	/* Starts and stops measurement. */
	void startMeasurement(void);
	
	/* Stops measurement. */
	void stopMeasurement(void);
	
	/* Recalibrates gyrsopcope. */
	void recalibrate(void);
	
	/* Starts recording data. */
	void recordData(void);
	
	/* Updates data of currently selected LPMS. */
	void updateCurrentLpms(QTreeWidgetItem *current = 0, QTreeWidgetItem *previous = 0);
		
	/* Starts magnetometer calibration. */
	void calibrateMag(void);
	
	/* Saves current calibration data to sensor. */
	void saveCalibration(void);
		
	/* Connects to OpenMAT server. */
	void startServer(void);

	/* Updates currently displayed FPS. */
	void updateLpmsFps(int v, int lpmsId);
	
	/* Updates all sensor status data. */
	void timerUpdate(void);
		
	/* Starts uploading a new firmware file. */
	void uploadFirmware(void);

	/* Starts uploading a new IAP file. */
	void uploadIap(void);

	/* Initilizes application menu bar. */
	void createMenuAndToolbar(void);
	
	/* Updates the upload timer status (firmware or IAP). */
	void uploadTimerUpdate(void);
	
	/* Switches self-test on and off. */
	void toggleSelfTest(void);
	
	/* Measures sensor latency. */
	void measureLatency(void);
	
	/* Acquires the current magnetic field map. */
	void getFieldMap(void);
	
	/* Switches to magnetic field map window. */
	void selectFieldMapWindow(void);
	
	/* Retrieves the current software / firmware version. */
	void getVersionInfo(void);
	
	/* Handles close application event. */
	void closeEvent(QCloseEvent *event);
	
	/* Resets to factory settings. */
	void resetToFactory(void);
	
	/* Starts accelerometer misalignment calibration. */
	void misalignmentCal(void);
	
	/* Displays new page for misalignment calibration. */
	void maNewPage(int i);
	
	/* Finishes misalignment calibration. */
	void maFinished(int i);
	
	/* Creates misalignment finished wizard page. */
	QWizardPage *maFinishedPage(void);
	
	/* Starts gyroscope misalignment calibration. */
	void gyrMisalignmentCal(void);
	
	/* Displays gyroscope misalignment wizard new page. */
	void gyrMaNewPage(int i);
	
	/* Finishes gyroscope misalignment calibration. */
	void gyrMaFinished(int i);
	
	/* Displays gyroscope misalignment finished page. */
	QWizardPage *gyrMaFinishedPage(void);
	
	/* Handles calibration timer update. */
	void calTimerUpdate(void);
	
	/* Starts wait bar with duration t. */
	void startWaitBar(int t);
	
	/* Opens add / remove devices dialog. */
	void addRemoveDevices(void);
	
	/* Saves calibration data. */
	void saveCalibrationData(void);
	
	/* Loads calibration data. */
	void loadCalibrationData(void);
	
	/* Selects cube mode 1 (one cube). */
	void selectCubeMode1(void);
	
	/* Selects cube mode 2 (two cubes). */
	void selectCubeMode2(void);
	
	/* Selects cube mode 4 (four cubes). */
	void selectCubeMode4(void);
	
	/* Browses record filename. */
	void browseRecordFile(void);
	
	/* Zeros reference of selected sensor. */
	void zeroReferenceSelected(void);

	/* Zeros reference of all sensors. */
	void zeroReferenceAll(void);
	
	/* Zeros selected angle offset. */
	void zeroAngleSelected(void);
	
	/* Zeros all angle offsets. */
	void zeroAngleAll(void);
	
	/* Initiates actual zeroing. */
	void resetTbSelected(void);
	
	/* Updates magnetic field map. */
	void updateMagneticFieldMap(void);
	
	/* Updates current baudrate to selected value. */
	void updateCanBaudrate(int i);
	
	/* Displays the heave motion graph. */
	void selectHeaveMotionWindow(void);
	
	/* Cancel firmware or IAP upload. */
	void cancelUpload(void);
	
	/* Checks if optional features are available for the current sensor. */
	void checkOptionalFeatures(LpmsSensorI* sensor);

	void selectGaitTrackingWindow(void);
	
private:
	QList<QTreeWidgetItem *> lpmsTreeItems;
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
	QComboBox *resetCombo;
	QComboBox *targetCombo;
	bool recordFileSet;
	MicroMeasure mm;
	QComboBox *canBaudrateList;
	QToolBar *toolbar;
	QMenu* viewMenu;
	bool heaveMotionEnabled;
	bool gaitTrackingEnabled;
	GaitTrackingWindow *gaitTrackingWindow;
	
#ifdef USE_ZEROC_ICE
	IceStormPublisher *isp;		
#endif

#ifdef USE_MB_SERVER
	MotionBuilderCommunication mbcom;
#endif
};

#endif
