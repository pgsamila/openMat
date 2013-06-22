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

#ifndef IMU_MONITOR_I
#define IMU_MONITOR_I

#include <QObject>

#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>

#include <boost/thread/thread.hpp> 

#include "ImuMonitor.h"
#include "ImuData.h"

#include <list>
using namespace std;

using namespace ImuDataCommunication;

class ImuMonitorI : public QObject, virtual public ImuMonitor
{
Q_OBJECT

	public:
		virtual void updateImuData(const OpenMatImuData& id, const Ice::Current&);
		
	signals:
		void newImuData(ImuData ld);
};

#endif