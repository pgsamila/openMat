cmake_minimum_required(VERSION 2.4.6)

project (LpSensorCWrapper)

if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

SET(CMAKE_BUILD_TYPE "Release")

include_directories("C:/OpenMAT/include")
link_directories("C:/OpenMAT/lib/x86")

ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
ADD_DEFINITIONS(-DDLL_EXPORT)

add_library(LpSensorCWrapper SHARED LpSensorCWrapper.cpp)

target_link_libraries(LpSensorCWrapper LpSensor.lib)