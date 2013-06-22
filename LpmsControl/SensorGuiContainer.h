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

#ifndef SENSOR_GUI_CONTAINER
#define SENSOR_GUI_CONTAINER 

#include <QTreeWidgetItem>
#include <QComboBox>
#include <QLabel>
#include <QGridLayout>
#include <QTreeWidgetItem>
#include <QTreeWidget>
#include <QCheckBox>
#include <QSpinBox>

#ifdef _WIN32
	#include "windows.h"
#endif
#include "LpmsSensorI.h"
#include "LpmsDefinitions.h"

#include <list>
#include <string>
#include <iostream>
using namespace std;

/*!
	\brief	Container for GUI elements related to one sensor unit.
	
	Its contents are used to build the sensor list tree in the control window.
*/
class SensorGuiContainer : QWidget 
{
	Q_OBJECT

public:
	QTreeWidgetItem* treeItem;
	QLabel* addressItem;
	QLabel* deviceTypeItem;		
	QLabel* parameterSetItem;
	QLabel* nameItem;
	QLabel* statusItem;
	QComboBox* indexItem;
	QLabel* runningItem;
	QLabel* firmwareItem;
	QLabel* thresholdEnableItem;
	QLabel* filterModeItem;
	QComboBox* filterModeCombo;
	QComboBox* parameterSetCombo;
	QComboBox* thresholdEnableCombo;
	QComboBox* gyrRangeCombo;
	QComboBox* accRangeCombo;
	QComboBox* magRangeCombo;
	QComboBox* gyrAutocalibrationCombo;
	QComboBox* canProtocolCombo;
	QComboBox* canBaudrateCombo;
	QComboBox* samplingRateCombo;
	QComboBox* magThreshEnableCombo;
	QCheckBox* selectAngularVelocity;
	QCheckBox* selectQuaternion;
	QCheckBox* selectEuler;
	QCheckBox* selectLinAcc;
	QCheckBox* selectGyro;
	QCheckBox* selectAcc;
	QCheckBox* selectMag;
	QCheckBox* selectPressure;
	QCheckBox* selectAltitude;
	QCheckBox* selectTemperature;
	QCheckBox* selectHeaveMotion;
	QComboBox* lowPassCombo;
	QComboBox* canTpdo1ACombo;
	QComboBox* canTpdo1BCombo;
	QComboBox* canTpdo2ACombo;
	QComboBox* canTpdo2BCombo;
	QComboBox* canTpdo3ACombo;
	QComboBox* canTpdo3BCombo;
	QComboBox* canTpdo4ACombo;
	QComboBox* canTpdo4BCombo;
	QComboBox* canHeartbeatCombo;
	QComboBox* linAccCompModeCombo;
	QComboBox* centriCompModeCombo;
	QComboBox* canChannelModeCombo;
	QComboBox* canPointModeCombo;	
	QSpinBox* canStartIdSpin;
	int openMatId;
	LpmsSensorI* sensor;
	QGridLayout* selectedDataGl;
	bool heaveMotionEnabled;
	int deviceType;
	
	SensorGuiContainer(LpmsSensorI* sensor, QTreeWidget* tree);
	~SensorGuiContainer(void);
	LpmsSensorI* getSensor(void);
	void checkOptionalFeatures(void);
	
public slots:
	void updateData(void);
	void updateGyrThresholdEnable(int i);
	void updateFilterPreset(int i);
	void updateFilterMode(int i);
	void updateGyrRange(int i);
	void updateAccRange(int i);
	void updateMagRange(int i);
	void updateOpenMATIndex(int i);
	void updateGyrAutocalibration(int i);
	void updateCanBaudrate(int i);	
	void updateCanProtocol(int i);
	void updatesamplingRate(int i);
	void updateSelectGyro(int i);	
	void updateSelectQuaternion(int i);
	void updateSelectEuler(int i);
	void updateSelectLinAcc(int i);
	void updateSelectAcc(int i);
	void updateSelectMag(int i);
	void updateSelectPressure(int i);
	void updateSelectAltitude(int i);
	void updateSelectTemperature(int i);
	void updateLowPass(int i);
	void updateSelectAngularVelocity(int i);
	void updateCanMapping(int i);
	void updateCanHeartbeat(int i);
	void updateSelectHeaveMotion(int i);
	void updateLinAccCompMode(int i);
	void updateCentriCompMode(int i);
	void updateCanStartId(int i);
	void updateCanPointMode(int i);
	void updateCanChannelMode(int i);
};

#endif