#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub --once /Sphinx/reset std_msgs/Bool '{data: true}'

