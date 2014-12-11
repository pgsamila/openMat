cmake_minimum_required(VERSION 2.4.6)

project (LpmsSimpleExample)

if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

SET(CMAKE_BUILD_TYPE "Debug")

set(BUILD_ARCHITECTURE "32-bit" CACHE STRING "")
set_property(CACHE BUILD_ARCHITECTURE PROPERTY STRINGS "32-bit" "64-bit")

if (BUILD_ARCHITECTURE STREQUAL "32-bit") 
include_directories("C:/OpenMAT/OpenMAT-1.3.0/include")
  link_directories("C:/OpenMAT/OpenMAT-1.3.0/lib/x86")
endif()
if (BUILD_ARCHITECTURE STREQUAL "64-bit") 
include_directories("C:/OpenMAT/OpenMAT-1.3.0-x64/include")
  link_directories("C:/OpenMAT/OpenMAT-1.3.0-x64/lib/x64")
endif()

ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

set(sources
	main.cpp
)

add_executable(LpmsSimpleExample main.cpp)

if (BUILD_ARCHITECTURE STREQUAL "32-bit") 
target_link_libraries(LpmsSimpleExample LpSensorD.lib)
endif()
if (BUILD_ARCHITECTURE STREQUAL "64-bit") 
target_link_libraries(LpmsSimpleExample LpSensorD64.lib)
endif()