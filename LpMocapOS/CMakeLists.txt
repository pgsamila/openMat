#/****************************************************************************
#**
#** Copyright (C) 2011 LP-Research
#** All rights reserved.
#** Contact: LP-Research (klaus@lp-research.com)
#**
#** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
#**
#** OpenMAT is free software: you can redistribute it and/or modify it under 
#** the terms of the GNU General Public License as published by the Free 
#** Software Foundation, either version 3 of the License, or (at your option) 
#** any later version.
#** 
#** OpenMAT is distributed in the hope that it will be useful, but WITHOUT 
#** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
#** FITNESS FOR A PARTICULAR PURPOSE. See the GNU \ General Public License 
#** for more details.
#** 
#** You should have received a copy of the GNU General Public License along 
#** with the OpenMAT library. If not, see <http://www.gnu.org/licenses/>.
#**
#****************************************************************************/

cmake_minimum_required(VERSION 2.4.6)

project (HumanBodySimulation)

if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

SET(USE_ZEROC_ICE TRUE)

if (USE_ZEROC_ICE)
	ADD_DEFINITIONS(-DUSE_ZEROC_ICE)
endif()	

set(LP_DROPBOX_PATH "C:/Users/user/Dropbox")

SET(CMAKE_BUILD_TYPE "Debug")

include_directories("C:/qwt-6.0.1/src")
include_directories("C:/Program Files/boost/boost_1_47")
include_directories("C:/Program Files/ZeroC/Ice-3.4.2/include")
include_directories("C:/eigen-eigen-3.0.2")
include_directories("../OpenMATCommon")
include_directories("../LpAlgorithm")

link_directories("C:/qwt-6.0.1/lib")
link_directories("C:/Program Files/boost/boost_1_47/lib")
link_directories("C:/Program Files (x86)/boost/boost_1_47/lib")
link_directories("C:/Program Files/ZeroC/Ice-3.4.2/lib/vc100")

include_directories("../Libraries/Qwt/include")
include_directories("../Libraries/Boost")
include_directories("../Libraries/Eigen")
include_directories("../Libraries/Ice/include")
include_directories("../OpenMATCommon")
include_directories("../LpAlgorithm")
include_directories("../LpSensor")
include_directories("../Libraries/Pugixml/src/")
include_directories("${LP_DROPBOX_PATH}/LpResearch/ProductDevelopment/Software/Libraries/Ice/include")

link_directories("../Libraries/Qwt/lib")
link_directories("../Libraries/Boost/lib")
link_directories("../Libraries/Ice/lib")
link_directories("../LpAlgorithm/build")
link_directories("${LP_DROPBOX_PATH}/LpResearch/ProductDevelopment/Software/Libraries/Ice/lib")

ADD_DEFINITIONS(-DQWT_DLL)
ADD_DEFINITIONS(-DNOMINMAX)
ADD_DEFINITIONS(-DEIGEN_DONT_ALIGN_STATICALLY)
ADD_DEFINITIONS(-D_WIN32_WINNT=0x05010200)
ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN)

set(sources
	main.cpp
	MainWindow.cpp
	HumanModel.cpp
	HumanModelWindow.cpp
	MicroMeasureWindows.cpp
	PlayControl.cpp
	GraphWindow.cpp
	Plot.cpp
#	HumanBodySimulation.rc

	../OpenMATCommon/ImuMonitor.cpp
	../OpenMATCommon/ImuMonitorI.cpp	
	../OpenMATCommon/IceStormSubscriber.cpp	
)
  
set(headers
	MainWindow.h
	HumanModel.h
	HumanModelWindow.h
	PlayControl.h
	MicroMeasure.h
	GraphWindow.h
	Plot.h

	../OpenMATCommon/LinkJoint.h
	../OpenMATCommon/ImuMonitor.h	
	../OpenMATCommon/ImuMonitorI.h		
	../OpenMATCommon/IceStormSubscriber.h

	../LpAlgorithm/LpAlgorithm.h	
)

set(EXECUTABLE_OUTPUT_PATH ./build)

find_package(OpenGL REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL Phonon QtXml REQUIRED)
include(${QT_USE_FILE})
qt4_wrap_cpp(moc_sources ${headers})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

add_executable(HumanBodySimulation ${sources} ${headers} ${moc_sources})
target_link_libraries(HumanBodySimulation ${QT_LIBRARIES})
target_link_libraries(HumanBodySimulation ${OPENGL_LIBRARIES})

target_link_libraries(HumanBodySimulation iced.lib icestormd.lib iceutild.lib)
target_link_libraries(HumanBodySimulation qwtd.lib)
target_link_libraries(HumanBodySimulation Ws2_32.lib)
# target_link_libraries(HumanBodySimulation LpAlgorithm.lib)