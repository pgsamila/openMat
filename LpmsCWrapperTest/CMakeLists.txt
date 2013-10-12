cmake_minimum_required(VERSION 2.4.6)

project (LpSensorCWrapperTest)

if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

set(BUILD_ARCHITECTURE "32-bit" CACHE STRING "")
set_property(CACHE BUILD_ARCHITECTURE PROPERTY STRINGS "32-bit" "64-bit")

SET(CMAKE_BUILD_TYPE "Release")

include_directories("../LpSensorCWrapper")
include_directories("C:/OpenMAT/include")

link_directories("../LpSensorCWrapper")
link_directories("../LpSensor/build")

ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

set(sources
	main.cpp
)

add_executable(LpSensorCWrapperTest main.cpp)

if (BUILD_ARCHITECTURE STREQUAL "32-bit")
	target_link_libraries(LpSensorCWrapperTest LpSensorCWrapper.lib)
endif()

if (BUILD_ARCHITECTURE STREQUAL "64-bit")
	target_link_libraries(LpSensorCWrapperTest LpSensorCWrapper64.lib)
endif()