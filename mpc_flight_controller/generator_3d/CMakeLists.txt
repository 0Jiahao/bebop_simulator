#
# Project settings
#

# Minimum required version of cmake 
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

set(CMAKE_CXX_STANDARD 11)

# Project name and programming languages used
PROJECT( mpc_flight_controller CXX )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#FILE(GLOB_RECURSE HDRS_FILES "src/*.h" "src/*.hpp")
#FILE(GLOB_RECURSE SRCS_FILES "src/*.c" "src/*.cpp")

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )

#
# Build an executable
#
ADD_EXECUTABLE(        ${PROJECT_NAME} src/mpc_flight_controller.cpp)
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES( ${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
