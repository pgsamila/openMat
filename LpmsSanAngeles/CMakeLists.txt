#/***********************************************************************
#** Copyright (C) 2011 LP-Research
#** All rights reserved.
#** Contact: LP-Research (klaus@lp-research.com)
#**
#** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
#**
#** Redistribution and use in source and binary forms, with 
#** or without modification, are permitted provided that the 
#** following conditions are met:
#**
#** Redistributions of source code must retain the above copyright 
#** notice, this list of conditions and the following disclaimer.
#** Redistributions in binary form must reproduce the above copyright 
#** notice, this list of conditions and the following disclaimer in 
#** the documentation and/or other materials provided with the 
#** distribution.
#**
#** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
#** FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
#** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
#** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#***********************************************************************/

cmake_minimum_required(VERSION 2.4.6)

project (LpmsSanAngeles)

if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")

include_directories("../Libraries/Qwt/include")
include_directories("../Libraries/Boost")
include_directories("../Lp3rdPartyLibraries/Eigen")
include_directories("../OpenMATCommon")
include_directories("../LpSensor")
include_directories("../Libraries/Pugixml/src/")
include_directories("C:/Program Files/Microsoft SDKs/Kinect/v1.5/inc")

link_directories("../Libraries/Qwt/lib")
link_directories("../Libraries/Boost/lib")
link_directories("C:/Program Files/boost/boost_1_47/lib")
link_directories("C:/Program Files (x86)/boost/boost_1_47/lib")
link_directories("C:/Program Files/Micr	osoft SDKs/Windows/v6.0A/Lib")
link_directories("../LpSensor/build")
link_directories("C:/Program Files/Microsoft SDKs/Kinect/v1.5/lib/x86")

ADD_DEFINITIONS(-DUSE_EIGEN)
ADD_DEFINITIONS(-DQWT_DLL)
ADD_DEFINITIONS(-DNOMINMAX)
ADD_DEFINITIONS(-DEIGEN_DONT_ALIGN_STATICALLY)
ADD_DEFINITIONS(-D_WIN32_WINNT=0x05010200)
# ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN)
# ADD_DEFINITIONS(-DINC_OLE2)
ADD_DEFINITIONS(-DPLATFORM_X86)
ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast /O2")
endif()

set(sources
	MainWindow.cpp
	SanAngelesWindow.cpp
	LpMatrix.c
	
	main.cpp
	
	LpmsSanAngeles.rc
)
  
set(headers
	MainWindow.h
	SanAngelesWindow.h
)

set(EXECUTABLE_OUTPUT_PATH ./build)

find_package(OpenGL REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL QtXml REQUIRED)
include(${QT_USE_FILE})
qt4_wrap_cpp(moc_sources ${headers})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

add_executable(LpmsSanAngeles ${sources} ${headers} ${moc_sources})
target_link_libraries(LpmsSanAngeles ${QT_LIBRARIES})
target_link_libraries(LpmsSanAngeles ${OPENGL_LIBRARIES})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
	target_link_libraries(LpmsSanAngeles LpSensor.lib)
	target_link_libraries(LpmsSanAngeles Ws2_32.lib)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	target_link_libraries(LpmsSanAngeles LpSensorD.lib)
	target_link_libraries(LpmsSanAngeles Ws2_32.lib)
endif()