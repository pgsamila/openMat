cmake_minimum_required(VERSION 2.4.6)

if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

set(BUILD_ARCHITECTURE "32-bit" CACHE STRING "")
set_property(CACHE BUILD_ARCHITECTURE PROPERTY STRINGS "32-bit" "64-bit")

SET(CMAKE_BUILD_TYPE "Release")

include_directories("../LpSensor")
include_directories("../OpenMATCommon")
link_directories("../LpSensor/build")

ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
ADD_DEFINITIONS(-DDLL_EXPORT)

if (BUILD_ARCHITECTURE STREQUAL "32-bit")
	project (LpSensorCWrapper)
	add_library(LpSensorCWrapper SHARED LpSensorCWrapper.cpp)
	target_link_libraries(LpSensorCWrapper LpSensor.lib)
endif()

if (BUILD_ARCHITECTURE STREQUAL "64-bit")
	project (LpSensorCWrapper64)
	add_library(LpSensorCWrapper64 SHARED LpSensorCWrapper.cpp)
	target_link_libraries(LpSensorCWrapper64 LpSensor64.lib)
endif()