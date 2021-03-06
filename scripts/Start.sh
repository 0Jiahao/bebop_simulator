#!/bin/bash

# clean up
sudo killall -9 roscore
sudo killall -9 rosmaster

# start Sphinx
gnome-terminal --tab "sphinx" -x bash -c "sudo systemctl start firmwared.service;GAZEBO_PLUGIN_PATH=`pwd`/build:$GAZEBO_PLUGIN_PATH sphinx --log-level=dbg /opt/parrot-sphinx/usr/share/sphinx/worlds/empty.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone;exec bash;"
sleep 10s

# start bebop autonomy
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal --tab "bebop_autonomy" -x bash -c "roslaunch bebop_driver bebop_node_sim.launch namespace:=\"bebop2\" drone_type:=\"bebop2\";exec bash;"
sleep 5s

# start ground control
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal --tab "groundcontrol" -x bash -c "roslaunch ground_control joystick_node_sim.launch;exec bash;"
sleep 1s

# start data logger
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal --tab "data logger" -x bash -c "rosrun sphinx_data_logger sphinx_data_logger;exec bash;"
sleep 1s

# start data logger
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal --tab "pub_15_hz" -x bash -c "rosrun topic_tools throttle messages /simulator/odometry 15 /bebop2/vio/odom;exec bash;"
sleep 1s

