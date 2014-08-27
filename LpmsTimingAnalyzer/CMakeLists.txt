cmake_minimum_required(VERSION 2.4.6)

project (LpmsTimingAnalyzer)

if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

SET(CMAKE_BUILD_TYPE "Release")
include_directories("../LpSensor")
link_directories("../LpSensor/build")

set(WINDOWS_JUSTTHREAD_PATH "C:/Program Files (x86)/JustSoftwareSolutions/JustThread" CACHE STRING "")
include_directories("${WINDOWS_JUSTTHREAD_PATH}/include")
link_directories("${WINDOWS_JUSTTHREAD_PATH}/lib")

ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

set(headers
	MicroMeasure.h
)
set(sources
	main.cpp
	MicroMeasureWindows.cpp
)

add_executable(${PROJECT_NAME} ${sources} ${headers})

target_link_libraries(${PROJECT_NAME} LpSensor.lib)
target_link_libraries(${PROJECT_NAME} justthread_vc10_mt.lib)