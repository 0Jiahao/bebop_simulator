# bebop_simulator
A simple bebop simulator for high level controller development. It has been tested on Ubuntu 16.04 (Xenial) with ROS Kinetic.
#### Prerequisite
- [Sphinx](https://developer.parrot.com/docs/sphinx/whatissphinx.html) *Note: you might need to disable the front camera and change the virtual ethernet.*
- [bebop_autonomy](https://bebop-autonomy.readthedocs.io/en/latest/) *Note: remember to change the IP address.*
- [Gazebo](http://gazebosim.org/)
- [nav_msgs](http://wiki.ros.org/nav_msgs)

#### Installation
```
$ cd [ROS_workspace]/src
$ git clone https://github.com/0Jiahao/bebop_simulator.git
$ cd ..
$ catkin build
```

#### Scripts
- `Start.sh`: start the simulation
- `Takeoff.sh`: takeoff
- `Land.sh`: land
- `Reset.sh`: start at the origin and takeoff
- `Random_Reset.sh`: start at a random position with random yaw angle and takeoff

#### Data reading
The ground truth data can be read via topic /simulator/odometry.
<img src="img/example.png" alt="fig1">