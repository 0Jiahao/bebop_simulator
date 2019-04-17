#
# Project settings
#

# Minimum required version of cmake 
CMAKE_MINIMUM_REQUIRED( VERSION 2.8.3 )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Project name and programming languages used
PROJECT( mpc_flight_controller_test)

include_directories(exportFolder/)
link_directories(exportFolder)
ADD_SUBDIRECTORY(exportFolder)

include_directories(qpoases)
link_directories(qpoases)
ADD_SUBDIRECTORY(qpoases)

ADD_EXECUTABLE(mpc_flight_controller_test test.cpp)
TARGET_LINK_LIBRARIES(mpc_flight_controller_test qpoases exportFolder)

