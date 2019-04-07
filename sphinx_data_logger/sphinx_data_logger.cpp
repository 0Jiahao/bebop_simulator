#include "ros/ros.h"

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/poses_stamped.pb.h>
#include "nav_msgs/Odometry.h"

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;

class data_logger
{
	public:
		gazebo::transport::SubscriberPtr odom_gazebo_sub;
		ros::Publisher odom_pub;

	data_logger(){}
	void read_pose(ConstPosesStampedPtr &msg);
};

// Reading Follower pose from Gazebo
void data_logger::read_pose(ConstPosesStampedPtr &msg){

	for (int i =0; i < msg->pose_size(); ++i)
    	{
		const ::gazebo::msgs::Pose &pose = msg->pose(i);
		if (pose.name() == "bebop2")
		{
			const ::gazebo::msgs::Vector3d &position = pose.position();
			const ::gazebo::msgs::Quaternion q = pose.orientation();
			nav_msgs::Odometry odom;
			odom.header.stamp = ros::Time::now();
			odom.header.frame_id = "world";
			odom.pose.pose.position.x = position.x();
			odom.pose.pose.position.y = position.y();
			odom.pose.pose.position.z = position.z();
			odom.pose.pose.orientation.x = q.x();
			odom.pose.pose.orientation.y = q.y();
			odom.pose.pose.orientation.z = q.z();
			odom.pose.pose.orientation.w = q.w();
			this->odom_pub.publish(odom);
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sphinx_data_logger");
	ros::NodeHandle nh;
	ros::Rate loop_rate(30);  // in Hz

	data_logger my_logger;

	gazebo::client::setup(argc, argv);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	my_logger.odom_gazebo_sub = node->Subscribe("/gazebo/default/pose/info", &data_logger::read_pose, &my_logger);
    my_logger.odom_pub = nh.advertise<nav_msgs::Odometry>("/groundTruth/odometry",1);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	gazebo::client::shutdown();
	return 0;
}


