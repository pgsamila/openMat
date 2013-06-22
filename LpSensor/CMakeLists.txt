#***********************************************************************
#* Copyright (C) 2013 LP-Research
#* All rights reserved.
#* Contact: LP-Research (info@lp-research.com)
#***********************************************************************

cmake_minimum_required(VERSION 2.4.6)

# Specifies build type (Debug or Release)
SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_BUILD_TYPE "Release")

# Defines path of dependencies for LpSensor library
# LpSensor has the following dependencies:
# FTDI D2XX drivers (http://www.ftdichip.com/Drivers/D2XX.htm)
# PCAN Basic API for CAN bus adapter library (http://www.peak-system.com/Support.55.0.html)
# Boost library (http://www.boost.org/)
# Windows SDK 6 (http://msdn.microsoft.com/en-us/dd299405.aspx)
# PugiXML (http://pugixml.org/)
set(OPENMAT_DEPENDENCIES_PATH "C:/Users/user/Dropbox/LpResearch/ProductDevelopment/Software/Libraries/")

# Selects the architecture to build for
# set(ARCH_64BIT TRUE)
set(ARCH_32BIT TRUE)

# Selects if ZeroC ICE network driver is to be used
# SET(USE_ZEROC_ICE TRUE)

# Selects if open-source version of the library is to be built
# SET(BUILD_OPENSOURCE TRUE)

if (ARCH_32BIT)
	if(CMAKE_BUILD_TYPE STREQUAL "Release")
		project (LpSensor)
	endif()

	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
		project (LpSensorD)
	endif()
endif()

if (ARCH_64BIT)
	if(CMAKE_BUILD_TYPE STREQUAL "Release")
		project (LpSensor64)
	endif()

	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
		project (LpSensorD64)
	endif()
endif()

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
	cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

if (USE_ZEROC_ICE)
	ADD_DEFINITIONS(-DUSE_ZEROC_ICE)
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	include_directories("../lp3rdpartylibraries/Ftdi")
	include_directories("../openmatcommon")
	include_directories("../lpsensor")
	include_directories("../lp3rdpartylibraries/Pugixml/src")
	include_directories("../lp3rdpartylibraries/PcanApi/Include")
	include_directories("../lp3rdpartylibraries/Boost")

	if (ARCH_32BIT)
		link_directories("${OPENMAT_DEPENDENCIES_PATH}/Ftdi/i386")
		link_directories("${OPENMAT_DEPENDENCIES_PATH}/PcanApi/Win32/VC_LIB")
		link_directories("${OPENMAT_DEPENDENCIES_PATH}/Boost/lib/win32")
		link_directories("${OPENMAT_DEPENDENCIES_PATH}Libraries/WindowsSdk6/Lib")
	endif()	
	
	if (ARCH_64BIT)
		link_directories("${OPENMAT_DEPENDENCIES_PATH}/Ftdi/amd64")
		link_directories("${OPENMAT_DEPENDENCIES_PATH}/PcanApi/x64/VC_LIB")	
		link_directories("${OPENMAT_DEPENDENCIES_PATH}/Boost/lib/win64")
		link_directories("${OPENMAT_DEPENDENCIES_PATH}/WindowsSdk6/Lib/x64")		
	endif()
	
	ADD_DEFINITIONS(-DUSE_EIGEN)
	ADD_DEFINITIONS(-DNOMINMAX)
	ADD_DEFINITIONS(-DEIGEN_DONT_ALIGN_STATICALLY)
	ADD_DEFINITIONS(-D_WIN32_WINNT=0x05010200)
	ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN)
	ADD_DEFINITIONS(-DPLATFORM_X86)
	ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
	ADD_DEFINITIONS(-DDLL_EXPORT)

	if(CMAKE_BUILD_TYPE STREQUAL "Release")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast /O2")
	endif()
	
	if (USE_ZEROC_ICE)
		include_directories("${OPENMAT_DEPENDENCIES_PATH}/Ice/include")
	endif()
	
	if (USE_ZEROC_ICE)
		link_directories("${OPENMAT_DEPENDENCIES_PATH}/Ice/lib")
	endif() 
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	include_directories("../openmatcommon")
	include_directories("../lp3rdpartylibraries/Pugixml/src")
	include_directories("../lp3rdpartylibraries/Ftdi/Linux/release")
	include_directories("/usr/local/include")
	include_directories("/usr/include")

	link_directories("/usr/lib")
	link_directories("/usr/lib/x86_64-linux-gnu")
	link_directories("/usr/local/lib")
	link_directories("${OPENMAT_DEPENDENCIES_PATH}/Ftdi/Linux/release/build/x86_64")
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(sources
		MicroMeasureWindows.cpp
		CanEngine.cpp	
		CalibrationData.cpp
		LpmsSensorManager.cpp
		LpmsBBluetooth.cpp	
		LpmsCanIo.cpp
		LpmsIoInterface.cpp	
		LpmsU.cpp	
		LpmsRS232.cpp
		LpmsSensor.cpp	
		LpMatrix.c

		PCANBasicClass.cpp
			
		../lp3rdpartylibraries/Pugixml/src/pugixml.cpp
	)
	
	if (BUILD_OPENSOURCE)	
		set(sources
			${sources}	
			LpFilterCVersionOS.c
			LpMagnetometerCalibrationOS.c
			CalcMisalignmentOS.c
			GaitTrackingOS.cpp
		)
	else()
		set(sources
			${sources}	
			../../openmat-closedsource/LpFilterCVersion.c
			../../openmat-closedsource/LpMagnetometerCalibration.c
			../../openmat-closedsource/CalcMisalignment.c
			../../openmat-closedsource/GaitTracking.cpp
		)
	endif()
	
	if (USE_ZEROC_ICE)
		set(sources
			${sources}
			IceImuDriver.cpp
		)
	endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	set(sources
		CanEngineLinux.cpp	
		MicroMeasure.cpp
		CalibrationData.cpp
		LpmsSensorManager.cpp
		LpmsBBluetoothLinux.cpp	
		LpmsCanIoLinux.cpp
		LpmsIoInterface.cpp	
		LpmsU.cpp	
		LpmsRS232Linux.cpp
		LpmsSensor.cpp	
		LpMatrix.c
						
		../lp3rdpartylibraries/Pugixml/src/pugixml.cpp
	)

	if (BUILD_OPENSOURCE)	
		set(sources
			${sources}	
			LpFilterCVersionOS.c
			LpMagnetometerCalibrationOS.c
			CalcMisalignmentOS.c
			GaitTrackingOS.cpp
		)
	else()
		set(sources
			${sources}	
			../../openmat-closedsource/LpFilterCVersion.c
			../../openmat-closedsource/LpMagnetometerCalibration.c
			../../openmat-closedsource/CalcMisalignment.c
			../../openmat-closedsource/GaitTracking.cpp
		)
	endif()

	if (USE_ZEROC_ICE)
		set(sources
			${sources}
			IceImuDriver.cpp
		)
	endif()
endif()
  
set(headers
)

if (ARCH_32BIT)
	set(LIBRARY_OUTPUT_PATH ./build)
endif()

if (ARCH_64BIT)
	set(LIBRARY_OUTPUT_PATH ./build/x64)
endif()

if (USE_ZEROC_ICE)
	find_package(Qt4 COMPONENTS QtCore REQUIRED)
	include(${QT_USE_FILE})
	qt4_wrap_cpp(moc_sources ${headers})
	include_directories(${CMAKE_BINARY_DIR})
	include_directories(${QT_INCLUDE_DIR})
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	if (ARCH_32BIT)
		if(CMAKE_BUILD_TYPE STREQUAL "Release")
			add_library(LpSensor SHARED ${sources} ${headers})
			target_link_libraries(LpSensor Ws2_32.lib)
			target_link_libraries(LpSensor ftd2xx.lib)
			target_link_libraries(LpSensor Bthprops.lib)
					
			if (USE_ZEROC_ICE)
				target_link_libraries(LpSensor ice.lib icestorm.lib iceutil.lib)
			endif()
		endif()

		if(CMAKE_BUILD_TYPE STREQUAL "Debug")
			add_library(LpSensorD SHARED ${sources} ${headers})
			target_link_libraries(LpSensorD Ws2_32.lib)
			target_link_libraries(LpSensorD ftd2xx.lib)
			target_link_libraries(LpSensorD Bthprops.lib)	

			if (USE_ZEROC_ICE)	
				target_link_libraries(LpSensorD iced.lib icestormd.lib iceutild.lib)
			endif()
		endif()
	endif()
	
	if (ARCH_64BIT)
		if(CMAKE_BUILD_TYPE STREQUAL "Release")
			add_library(LpSensor64 SHARED ${sources} ${headers})
			target_link_libraries(LpSensor64 Ws2_32.lib)
			target_link_libraries(LpSensor64 ftd2xx.lib)
			target_link_libraries(LpSensor64 Bthprops.lib)
					
			if (USE_ZEROC_ICE)
				target_link_libraries(LpSensor64 ice.lib icestorm.lib iceutil.lib)
			endif()
		endif()

		if(CMAKE_BUILD_TYPE STREQUAL "Debug")
			add_library(LpSensorD64 SHARED ${sources} ${headers})
			target_link_libraries(LpSensorD64 Ws2_32.lib)
			target_link_libraries(LpSensorD64 ftd2xx.lib)
			target_link_libraries(LpSensorD64 Bthprops.lib)	

			if (USE_ZEROC_ICE)	
				target_link_libraries(LpSensorD64 iced.lib icestormd.lib iceutild.lib)
			endif()
		endif()
	endif()	
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if(CMAKE_BUILD_TYPE STREQUAL "Release")
		add_library(LpSensor SHARED ${sources} ${headers})
		target_link_libraries(LpSensor ftd2xx)
		target_link_libraries(LpSensor bluetooth)
	endif()

	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
		add_library(LpSensorD SHARED ${sources} ${headers})
		target_link_libraries(LpSensorD ftd2xx)
		target_link_libraries(LpSensorD bluetooth)
	endif()
endif()
