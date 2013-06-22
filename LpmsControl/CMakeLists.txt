#***********************************************************************
#* Copyright (C) 2013 LP-Research
#* All rights reserved.
#* Contact: LP-Research (info@lp-research.com)
#*
#* This file is part of the Open Motion Analysis Toolkit (OpenMAT).
#*
#* Redistribution and use in source and binary forms, with 
#* or without modification, are permitted provided that the 
#* following conditions are met:
#*
#* Redistributions of source code must retain the above copyright 
#* notice, this list of conditions and the following disclaimer.
#* Redistributions in binary form must reproduce the above copyright 
#* notice, this list of conditions and the following disclaimer in 
#* the documentation and/or other materials provided with the 
#* distribution.
#*
#* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
#* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#* HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
#* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
#* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#**********************************************************************/

cmake_minimum_required(VERSION 2.4.6)

project (LpmsControl)

if (COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

SET(USE_ZEROC_ICE FALSE)
SET(USE_QT5 FALSE)
SET(USE_QT4 TRUE)

SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_BUILD_TYPE "Release")

set(OPENMAT_DEPENDENCIES_PATH "C:/Users/user/Dropbox/LpResearch/ProductDevelopment/Software/Libraries/")

if (CMAKE_BUILD_TYPE STREQUAL "Release")
	SET(SHOW_CONSOLE FALSE)
	set(EXECUTABLE_OUTPUT_PATH ./build/release)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(SHOW_CONSOLE TRUE)
	set(EXECUTABLE_OUTPUT_PATH ./build/debug)	
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	include_directories("../lp3rdpartylibraries/Ftdi")
	include_directories("../lp3rdpartylibraries/Qwt/include")
	include_directories("../lp3rdpartylibraries/Boost")
	include_directories("../lp3rdpartylibraries/Eigen")

	if (USE_ZEROC_ICE)
		include_directories("../lp3rdpartylibraries/Ice/include")
	endif()

	include_directories("../OpenMATCommon")
	include_directories("../LpAlgorithm")
	include_directories("../LpSensor")
	include_directories("../lp3rdpartylibraries/Pugixml/src/")

	link_directories("${OPENMAT_DEPENDENCIES_PATH}/Qwt/lib")
	link_directories("${OPENMAT_DEPENDENCIES_PATH}/Boost/lib/win32")	

	if (USE_ZEROC_ICE)
		link_directories("${OPENMAT_DEPENDENCIES_PATH}/Ice/lib")
	endif() 

	link_directories("../LpSensor/build")

	ADD_DEFINITIONS(-DUSE_EIGEN)
	ADD_DEFINITIONS(-DQWT_DLL)
	ADD_DEFINITIONS(-DNOMINMAX)
	ADD_DEFINITIONS(-DEIGEN_DONT_ALIGN_STATICALLY)
	ADD_DEFINITIONS(-D_WIN32_WINNT=0x05010200)
	ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN)
	ADD_DEFINITIONS(-DPLATFORM_X86)
	ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

	if (SHOW_CONSOLE)
		ADD_DEFINITIONS(-DSHOW_CONSOLE)
	endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	SET(SHOW_CONSOLE TRUE)

	include_directories("../lp3rdpartylibraries/Qwt/include")
	include_directories("../lp3rdpartylibraries/Pugixml/src")
	include_directories("../lp3rdpartylibraries/Ftdi/Linux/release")
	include_directories("../lp3rdpartylibraries/Eigen")

	include_directories("../openmatcommon")
	include_directories("../lpsensor")

	include_directories("/usr/local/include")
	include_directories("/usr/include")

	link_directories("../lp3rdpartylibraries/Ftdi/Linux/release/build/x86_64")
	link_directories("../lpsensor/build")

	link_directories("/usr/lib")
	link_directories("/usr/lib/x86_64-linux-gnu")
	link_directories("/usr/local/lib")

	if (USE_ZEROC_ICE)
		link_directories("../lp3rdpartylibraries/Ice/lib")
	endif() 
endif()

if (USE_ZEROC_ICE)
	ADD_DEFINITIONS(-DUSE_ZEROC_ICE)
endif()	

if (${WIN32} MATCHES TRUE)
	if (CMAKE_BUILD_TYPE STREQUAL "Release")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast /O2")
	endif()
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(sources
		MainWindow.cpp
		Plot.cpp
		GraphWindow.cpp
		ThreeDWindow.cpp
		main.cpp
		StdRedirector.cpp
		SensorGuiContainer.cpp
		ObjFileParser.cpp
		DiscoveryItem.cpp
		DiscoveryTree.cpp
		FieldMapDisplay.cpp
		FieldMapContainer.cpp
		FieldModuleIndicator.cpp
		RescanDialog.cpp
		GaitTrackingWindow.cpp
	
		LpmsControl.rc

		../lp3rdpartylibraries/Pugixml/src/pugixml.cpp
	)

	if (USE_ZEROC_ICE)
		set(sources
			${sources}
			../OpenMATCommon/ImuMonitor.cpp
			../OpenMATCommon/RawImuMonitorI.cpp	
			../OpenMATCommon/IceStormCommunication.cpp
		)
	endif()
	 
	set(headers
		MainWindow.h
		Plot.h
		GraphWindow.h
		ThreeDWindow.h
		StdRedirector.h		
		SensorGuiContainer.h
		ObjFileParser.h
		DiscoveryItem.h
		DiscoveryTree.h
		FieldMapDisplay.h
		FieldMapContainer.h
		FieldModuleIndicator.h
		RescanDialog.h
		CubeWindowContainer.h
		GaitTrackingWindow.h
		VerticalBarGraph.h
	)

	if (USE_ZEROC_ICE)
		set(headers
			${headers}
			../OpenMATCommon/ImuMonitor.h
			../OpenMATCommon/RawImuMonitorI.h	
			../OpenMATCommon/IceStormCommunication.h
		)
	endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	set(sources
		MainWindow.cpp
		Plot.cpp
		GraphWindow.cpp
		ThreeDWindow.cpp
		main.cpp
		SensorGuiContainer.cpp
		ObjFileParser.cpp
		DiscoveryItem.cpp
		DiscoveryTree.cpp
		FieldMapDisplay.cpp
		FieldMapContainer.cpp
		FieldModuleIndicator.cpp
		RescanDialog.cpp
		GaitTrackingWindow.cpp
	
		LpmsControl.rc

		../lp3rdpartylibraries/Pugixml/src/pugixml.cpp
	)

	if (USE_ZEROC_ICE)
		set(sources
			${sources}
			../OpenMATCommon/ImuMonitor.cpp
			../OpenMATCommon/RawImuMonitorI.cpp	
			../OpenMATCommon/IceStormCommunication.cpp
		)
	endif()
	 
	set(headers
		MainWindow.h
		Plot.h
		GraphWindow.h
		ThreeDWindow.h
		SensorGuiContainer.h
		ObjFileParser.h
		DiscoveryItem.h
		DiscoveryTree.h
		FieldMapDisplay.h
		FieldMapContainer.h
		FieldModuleIndicator.h
		RescanDialog.h
		CubeWindowContainer.h
		GaitTrackingWindow.h
		VerticalBarGraph.h
	)

	if (USE_ZEROC_ICE)
		set(headers
			${headers}
			../OpenMATCommon/ImuMonitor.h
			../OpenMATCommon/RawImuMonitorI.h	
			../OpenMATCommon/IceStormCommunication.h
		)
	endif()
endif()

if (USE_QT4)
	find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
	include(${QT_USE_FILE})
	qt4_wrap_cpp(moc_sources ${headers})
	include_directories(${CMAKE_BINARY_DIR})
	include_directories(${QT_INCLUDE_DIR})
endif()

if (USE_QT5)
	set(CMAKE_INCLUDE_CURRENT_DIR ON)
	set(CMAKE_AUTOMOC ON)

	find_package(Qt5Widgets REQUIRED)
	find_package(Qt5Core REQUIRED)
	find_package(Qt5OpenGL REQUIRED)

	qt5_use_modules(LpmsControl Widgets Core OpenGL)
endif()

find_package(OpenGL REQUIRED)
	
if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	if (SHOW_CONSOLE)
		add_executable(LpmsControl ${sources} ${headers} ${moc_sources})
	else()
		add_executable(LpmsControl WIN32 ${sources} ${headers} ${moc_sources})
	endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	add_executable(LpmsControl ${sources} ${headers} ${moc_sources})
endif()
	
target_link_libraries(LpmsControl ${QT_LIBRARIES})
target_link_libraries(LpmsControl ${OPENGL_LIBRARIES})

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	if(CMAKE_BUILD_TYPE STREQUAL "Release")
		target_link_libraries(LpmsControl LpSensor.lib)
		target_link_libraries(LpmsControl qwt.lib)
		target_link_libraries(LpmsControl Ws2_32.lib)

		if (USE_ZEROC_ICE)
			target_link_libraries(LpmsControl ice.lib icestorm.lib iceutil.lib)
		endif()
	endif()

	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
		target_link_libraries(LpmsControl LpSensorD.lib)
		target_link_libraries(LpmsControl qwtd.lib)
		target_link_libraries(LpmsControl Ws2_32.lib)
	
		if (USE_ZEROC_ICE)	
			target_link_libraries(LpmsControl iced.lib icestormd.lib iceutild.lib)
		endif()	
	endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if(CMAKE_BUILD_TYPE STREQUAL "Release")
		target_link_libraries(LpmsControl libLpSensor.so)
		target_link_libraries(LpmsControl libqwt.so)
		target_link_libraries(LpmsControl libboost_thread.so)
	endif()

	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
		target_link_libraries(LpmsControl libLpSensorD.so)
		target_link_libraries(LpmsControl libqwt.so)		
		target_link_libraries(LpmsControl libboost_thread.so)
	endif()
endif()
