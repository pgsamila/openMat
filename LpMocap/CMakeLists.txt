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

project (LpMocap)

if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0015 NEW)
cmake_policy(SET CMP0020 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")

set(USE_JUST_THREAD CACHE BOOL "")

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
	set(WINDOWS_QT5_PATH "C:/Qt/Qt5.2.1/5.2.1/msvc2010_opengl" CACHE STRING "")
	set(WINDOWS_JUSTTHREAD_PATH "C:/Program Files (x86)/JustSoftwareSolutions/JustThread" CACHE STRING "")
	set(WINDOWS_FTDI_PATH "C:/ftdi" CACHE STRING "")
	set(WINDOWS_PUGIXML_PATH "C:/pugixml" CACHE STRING "")
	set(WINDOWS_EIGEN_PATH "C:/eigen-3.2.0")
	set(WINDOWS_OPENCV_PATH "C:/opencv" CACHE STRING "")
	set(WINDOWS_BOOST_PATH "C:/boost_1_55_0" CACHE STRING "")

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

	include_directories("${WINDOWS_OPENCV_PATH}/build/include")
	link_directories("${WINDOWS_OPENCV_PATH}/build/x86/vc10/lib")	
	
	link_directories("../openmat/LpSensor/build")

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

set(sources
	main.cpp
	MainWindow.cpp
	HumanModel.cpp
	HumanModelWindow.cpp
	MicroMeasureWindows.cpp
	PlayControl.cpp
	GraphWindow.cpp
	Plot.cpp
	pugixml.cpp
	MotionBuilderCommunication.cpp
	C3dParser.cpp
#	HumanBodySimulation.rc
)
  
set(headers
	MainWindow.h
	HumanModel.h
	HumanModelWindow.h
	PlayControl.h
	MicroMeasure.h
	GraphWindow.h
	Plot.h
	MotionBuilderCommunication.h
	MotionBuilderSocketInterface.h	
	LinkJoint.h
)

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
find_package(Qt5Xml REQUIRED)
include_directories(${Qt5Xml_INCLUDE_DIRS})

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	set_target_properties(Qt5::Core PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "DEBUG")
	set_target_properties(Qt5::Widgets PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "DEBUG")
	set_target_properties(Qt5::OpenGL PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "DEBUG")
	set_target_properties(Qt5::Xml PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "DEBUG")
endif()

find_package(OpenGL REQUIRED)
	
if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	if (SHOW_CONSOLE)
		add_executable(LpMocap ${sources} ${headers} ${moc_sources})
	else()
		add_executable(LpMocap WIN32 ${sources} ${headers} ${moc_sources})
	endif()
endif()
	
target_link_libraries(LpMocap ${QT_LIBRARIES})
target_link_libraries(LpMocap ${OPENGL_LIBRARIES})

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	if(CMAKE_BUILD_TYPE STREQUAL "Release")
		target_link_libraries(LpMocap Qt5::Widgets)		
		target_link_libraries(LpMocap Qt5::Core)
		target_link_libraries(LpMocap Qt5::OpenGL)
		target_link_libraries(LpMocap Qt5::Xml)
		target_link_libraries(LpMocap qwt.lib)
		target_link_libraries(LpMocap Ws2_32.lib)
		target_link_libraries(LpMocap opencv_core248.lib)		
		target_link_libraries(LpMocap opencv_highgui248.lib)
		target_link_libraries(LpMocap opencv_imgproc248.lib)
		target_link_libraries(LpMocap opencv_contrib248.lib)		
		
		if (USE_JUSTTHREAD)	
			target_link_libraries(LpMocap justthread_vc10_mt.lib)
		endif()	
	endif()

	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
		target_link_libraries(LpMocap Qt5::Widgets)
		target_link_libraries(LpMocap Qt5::Core)
		target_link_libraries(LpMocap Qt5::OpenGL)
		target_link_libraries(LpMocap Qt5::Xml)
		target_link_libraries(LpMocap qwtd.lib)
		target_link_libraries(LpMocap Ws2_32.lib)
		target_link_libraries(LpMocap opencv_core248d.lib)
		target_link_libraries(LpMocap opencv_highgui248d.lib)
		target_link_libraries(LpMocap opencv_imgproc248d.lib)
		target_link_libraries(LpMocap opencv_contrib248d.lib)		
	
		if (USE_JUSTTHREAD)	
			target_link_libraries(LpMocap justthread_vc10_mtd.lib)
		endif()	
	endif()
endif()