cmake_minimum_required(VERSION 2.4.6)

project (LpSensorCWrapperTest)

if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

SET(CMAKE_BUILD_TYPE "Release")

include_directories("../LpSensorCWrapper")
include_directories("C:/OpenMAT/include")

link_directories("../LpSensorCWrapper")
link_directories("C:/OpenMAT/lib/x86")

ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

set(sources
	main.cpp
)

add_executable(LpSensorCWrapperTest main.cpp)

target_link_libraries(LpSensorCWrapperTest LpSensorCWrapper.lib)