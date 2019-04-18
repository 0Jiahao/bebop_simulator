# What version of CMake is needed ?
cmake_minimum_required(VERSION 2.8.3 )

# Name of this package .
project( bebop_mpc )

# Find the catkin build system , and any other packages on
# which we depend .
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs tf)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Declare our catkin package .
catkin_package( )

# Specify locations of header files .
include_directories(${catkin_INCLUDE_DIRS})

include_directories(exportFolder)
link_directories(exportFolder)
add_subdirectory(ThirdParty/exportFolder)

include_directories(qpoases)
link_directories(qpoases)
add_subdirectory(ThirdParty/qpoases)
# Declare the executable , along with its sourcefiles . If
# there are multiple executables ,use multiple copies of
# this line .

add_executable( bebop_mpc_node bebop_mpc_node.cpp)
target_link_libraries( bebop_mpc_node ${catkin_LIBRARIES} qpoases exportFolder Eigen3::Eigen)

# Specify libraries against which to link. Again , this
# line should be copied for each distinct executable in
# the package .
#target_link_libraries( bebop_mpc_node ${catkin_LIBRARIES} Eigen3::Eigen)

 
