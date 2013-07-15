#***********************************************************************
#* Copyright (C) 2013 LP-Research
#* All rights reserved.
#* Contact: LP-Research (info@lp-research.com)
#***********************************************************************

cmake_minimum_required(VERSION 2.4.6)

set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")

set(BUILD_ARCHITECTURE "32-bit" CACHE STRING "")
set_property(CACHE BUILD_ARCHITECTURE PROPERTY STRINGS "32-bit" "64-bit")

set(USE_ICE FALSE CACHE BOOL "")
set(BUILD_OPENSOURCE CACHE BOOL "")

if (BUILD_ARCHITECTURE STREQUAL "32-bit")
	if(CMAKE_BUILD_TYPE STREQUAL "Release")
		project (LpSensor)
	endif()

	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
		project (LpSensorD)
	endif()
endif()

if (BUILD_ARCHITECTURE STREQUAL "64-bit")
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
	set(WINDOWS_BOOST_PATH "C:/Users/user/Dropbox/LpResearch/ProductDevelopment/Software/Libraries/Boost" CACHE STRING "")
	set(WINDOWS_ICE_PATH "C:/Users/user/Dropbox/LpResearch/ProductDevelopment/Software/Libraries/Ice" CACHE STRING "")
	set(WINDOWS_SDK_PATH "C:/Users/user/Dropbox/LpResearch/ProductDevelopment/Software/Libraries/WindowsSdk6" CACHE STRING "")
	set(WINDOWS_FTDI_PATH "C:/Users/user/Dropbox/LpResearch/ProductDevelopment/Software/Libraries/FTdi" CACHE STRING "")

	include_directories("../Lp3rdPartyLibraries/Ftdi")
	include_directories("../OpenMATCommon")
	include_directories("../LpSensor")
	include_directories("../Lp3rdPartyLibraries/Pugixml/src")
	include_directories("../Lp3rdPartyLibraries/PcanApi/Include")
	include_directories("${WINDOWS_BOOST_PATH}")

if (BUILD_ARCHITECTURE STREQUAL "32-bit")
		link_directories("${WINDOWS_FTDI_PATH}/i386")
		link_directories("${WINDOWS_PCAN_PATH}/Win32/VC_LIB")
		link_directories("${WINDOWS_BOOST_PATH}/lib/win32")
		link_directories("${WINDOWS_SDK_PATH}/Lib")
	endif()	
	
if (BUILD_ARCHITECTURE STREQUAL "64-bit")
		link_directories("${WINDOWS_FTDI_PATH}/amd64")
		link_directories("${WINDOWS_PCAN_PATH}/x64/VC_LIB")	
		link_directories("${WINDOWS_BOOST_PATH}/lib/win64")
		link_directories("${WINDOWS_SDK_PATH}/Lib/x64")		
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
	include_directories("../OpenMATCommon")
	include_directories("../Lp3rdPartyLibraries/Pugixml/src")
	include_directories("../Lp3rdPartyLibraries/Ftdi/Linux/release")
	
	include_directories("/usr/local/include")
	include_directories("/usr/include")

	link_directories("/usr/lib")
	link_directories("/usr/lib/x86_64-linux-gnu")
	link_directories("/usr/local/lib")
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
			
		../Lp3rdPartyLibraries/Pugixml/src/pugixml.cpp
	)
	
	if (BUILD_OPENSOURCE)	
		set(sources
			${sources}	
			LpMagnetometerCalibrationOS.c
			CalcMisalignmentOS.c
			GaitTrackingOS.cpp
		)
	else()
		set(sources
			${sources}	
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
						
		../Lp3rdPartyLibraries/Pugixml/src/pugixml.cpp
	)

	if (BUILD_OPENSOURCE)	
		set(sources
			${sources}
			LpMagnetometerCalibrationOS.c
			CalcMisalignmentOS.c
			GaitTrackingOS.cpp
		)
	else()
		set(sources
			${sources}	
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

if (BUILD_ARCHITECTURE STREQUAL "32-bit")
	set(LIBRARY_OUTPUT_PATH ./build CACHE STRING "")
	set(EXECUTABLE_OUTPUT_PATH ./build CACHE STRING "")
endif()

if (BUILD_ARCHITECTURE STREQUAL "64-bit")
	set(LIBRARY_OUTPUT_PATH ./build/x64 CACHE STRING "")
	set(EXECUTABLE_OUTPUT_PATH ./build/x64 CACHE STRING "")
endif()

if (USE_ZEROC_ICE)
	find_package(Qt4 COMPONENTS QtCore REQUIRED)
	include(${QT_USE_FILE})
	qt4_wrap_cpp(moc_sources ${headers})
	include_directories(${CMAKE_BINARY_DIR})
	include_directories(${QT_INCLUDE_DIR})
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	if (BUILD_ARCHITECTURE STREQUAL "32-bit")
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
	
	if (BUILD_ARCHITECTURE STREQUAL "64-bit")
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