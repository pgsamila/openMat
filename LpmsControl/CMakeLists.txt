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

cmake_minimum_required(VERSION 2.8.3)

project (LpmsControl)

if (COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
	cmake_policy(SET CMP0015 NEW)
	cmake_policy(SET CMP0020 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")

set(USE_JUST_THREAD CACHE BOOL "")
set(USE_ZEROC_ICE CACHE BOOL "")

if (CMAKE_BUILD_TYPE STREQUAL "Release")
	set(SHOW_CONSOLE TRUE)
	set(EXECUTABLE_OUTPUT_PATH "./build/release")
	set(LIBRARY_OUTPUT_PATH "./build/release")
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	set(SHOW_CONSOLE TRUE)
	set(EXECUTABLE_OUTPUT_PATH "./build/debug")	
	set(LIBRARY_OUTPUT_PATH "./build/debug")	
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(WINDOWS_QWT_PATH "C:/qwt-6.1.0" CACHE STRING "")
	set(WINDOWS_QT5_PATH "C:/Qt/Qt5.1.1/5.1.1/msvc2010_opengl" CACHE STRING "")
	set(WINDOWS_JUSTTHREAD_PATH "C:/Program Files (x86)/JustSoftwareSolutions/JustThread" CACHE STRING "")
	set(WINDOWS_FTDI_PATH "C:/ftdi" CACHE STRING "")
	set(WINDOWS_BOOST_PATH "C:/boost_1_55_0" CACHE STRING "")
	set(WINDOWS_PUGIXML_PATH "C:/pugixml" CACHE STRING "")
	set(WINDOWS_ICE_PATH "C:/Program Files (x86)/ZeroC/Ice-3.5.1" CACHE STRING "")
	set(WINDOWS_EIGEN_PATH "C:/eigen-3.2.0")

	include_directories("${WINDOWS_BOOST_PATH}")
	include_directories("${WINDOWS_PUGIXML_PATH}/src")
	include_directories("${WINDOWS_FTDI_PATH}")
	include_directories("${WINDOWS_EIGEN_PATH}")

	include_directories("../OpenMATCommon")
	include_directories("../LpSensor")

	include_directories("${WINDOWS_JUSTTHREAD_PATH}/include")
	link_directories("${WINDOWS_JUSTTHREAD_PATH}/lib")	
	
	include_directories("${WINDOWS_QWT_PATH}/src")
	link_directories("${WINDOWS_QWT_PATH}/lib")

	include_directories("${WINDOWS_ICE_PATH}/Ice/include")	
	link_directories("${WINDOWS_ICE_PATH}/Ice/lib")

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

	set(LINUX_PUGIXML_PATH "~/pugixml" CACHE STRING "")
	include_directories("${LINUX_PUGIXML_PATH}/src")

	set(LINUX_QWT_PATH "~/qwt-6.1/qwt-6.1.0" CACHE STRING "")
	include_directories("${LINUX_QWT_PATH}/src")
	link_directories("${LINUX_QWT_PATH}/lib")

	include_directories("/usr/include/eigen3")

	include_directories("../OpenMATCommon")
	include_directories("../LpSensor")
	link_directories("../LpSensor/build")

	include_directories("/usr/local/include")
	include_directories("/usr/include")

	if (BUILD_ARCHITECTURE STREQUAL "32-bit")
	endif()

	if (BUILD_ARCHITECTURE STREQUAL "64-bit")
		link_directories("/usr/lib/x86_64-linux-gnu")
	endif()

	link_directories("/usr/lib")
	link_directories("/usr/local/lib")

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++11 -fPIC")
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
		MotionBuilderCommunication.cpp
		PlayControl.cpp
		
		../LpSensor/LpMatrix.c
	
		LpmsControl.rc

		${WINDOWS_PUGIXML_PATH}/src/pugixml.cpp
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
		MotionBuilderCommunication.h
		PlayControl.h		
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
		MotionBuilderCommunication.cpp
	
		../LpSensor/LpMatrix.cpp	
	
		LpmsControl.rc

		${LINUX_PUGIXML_PATH}/src/pugixml.cpp
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
		MotionBuilderCommunication.h
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

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(CMAKE_PREFIX_PATH "${WINDOWS_QT5_PATH}")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets REQUIRED)
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDES})
add_definitions(${Qt5Widgets_DEFINITIONS})
find_package(Qt5Core REQUIRED)
include_directories(${Qt5Core_INCLUDE_DIRS})
find_package(Qt5OpenGL REQUIRED)
include_directories(${Qt5OpenGL_INCLUDE_DIRS})

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
		target_link_libraries(LpmsControl Qt5::Widgets)		
		target_link_libraries(LpmsControl Qt5::Core)
		target_link_libraries(LpmsControl Qt5::OpenGL)		
		target_link_libraries(LpmsControl qwt.lib)
		target_link_libraries(LpmsControl Ws2_32.lib)

		if (USE_JUSTTHREAD)	
			target_link_libraries(LpmsControl justthread_vc10_mt.lib)
		endif()	
		
		if (USE_ZEROC_ICE)
			target_link_libraries(LpmsControl ice.lib icestorm.lib iceutil.lib)
		endif()
	endif()

	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
		target_link_libraries(LpmsControl LpSensorD.lib)
		target_link_libraries(LpmsControl Qt5::Widgets)
		target_link_libraries(LpmsControl Qt5::Core)
		target_link_libraries(LpmsControl Qt5::OpenGL)
		target_link_libraries(LpmsControl qwtd.lib)
		target_link_libraries(LpmsControl Ws2_32.lib)
	
		if (USE_JUSTTHREAD)	
			target_link_libraries(LpmsControl justthread_vc10_mtd.lib)
		endif()	
	
		if (USE_ZEROC_ICE)	
			target_link_libraries(LpmsControl iced.lib icestormd.lib iceutild.lib)
		endif()	
	endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if(CMAKE_BUILD_TYPE STREQUAL "Release")
		target_link_libraries(LpmsControl libLpSensor.so)
		target_link_libraries(LpmsControl ${LINUX_QWT_PATH}/lib/libqwt.so)
		target_link_libraries(LpmsControl Qt5::Widgets)
		target_link_libraries(LpmsControl Qt5::Core)
		target_link_libraries(LpmsControl Qt5::OpenGL)
	endif()

	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
		target_link_libraries(LpmsControl libLpSensorD.so)
		target_link_libraries(LpmsControl ${LINUX_QWT_PATH}/lib/libqwt.so)
		target_link_libraries(LpmsControl Qt5::Widgets)
		target_link_libraries(LpmsControl Qt5::Core)
		target_link_libraries(LpmsControl Qt5::OpenGL)	
	endif()
endif()