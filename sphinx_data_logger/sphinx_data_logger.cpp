#include "ros/ros.h"

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/poses_stamped.pb.h>

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;

// Reading Follower pose from Gazebo
void Callback(ConstPosesStampedPtr &msg){

	for (int i =0; i < msg->pose_size(); ++i)
    	{
		const ::gazebo::msgs::Pose &pose = msg->pose(i);
		if (pose.name() == "bebop2")
		{
			const ::gazebo::msgs::Vector3d &position = pose.position();
			const ::gazebo::msgs::Quaternion q = pose.orientation();
			cout << position.x() << " " << position.y() << " " << position.z() << endl;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sphinx_data_logger");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);  // in Hz

	gazebo::client::setup(argc, argv);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	gazebo::transport::SubscriberPtr odom_gazebo_sub = node->Subscribe("/gazebo/default/pose/info", Callback);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	gazebo::client::shutdown();
	return 0;
}


