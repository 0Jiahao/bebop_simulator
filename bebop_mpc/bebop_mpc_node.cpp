#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define NX          ACADO_NX	/* number of differential states */
#define NXA         ACADO_NXA	/* number of alg. states */
#define NU          ACADO_NU	/* number of control inputs */
#define N          	ACADO_N		/* number of control intervals */
#define NY			ACADO_NY	/* number of references, nodes 0..N - 1 */
#define NYN			ACADO_NYN
#define M_PI 3.14159265358979323846

using namespace std;
using namespace Eigen;

// mpc variables
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class bebop_mpc
{
    public:
        // member
        ros::NodeHandle nh;   		            // define node
		ros::Subscriber odom_sub;               // mav state subscriber 
		ros::Publisher  ctrl_pub;               // control command publisher
        geometry_msgs::Pose ref;                // reference pose
        bebop_mpc();
        void read_state(const nav_msgs::Odometry& msg);
};

bebop_mpc::bebop_mpc()
{
    this->odom_sub = nh.subscribe("/bebop2/odometry", 1, &bebop_mpc::read_state,this);
    this->ctrl_pub = nh.advertise<geometry_msgs::Twist>("/bebop2/cmd_vel",1);
    this->ref.position.x = 3;
    this->ref.position.y = -3;
    this->ref.position.z = 1.25;
}

void bebop_mpc::read_state(const nav_msgs::Odometry& msg)
{
    // set reference
    for (int i = 0; i < NY * N; ++i)
	{
		acadoVariables.y[i * NY + 0] = 0; // x
		acadoVariables.y[i * NY + 1] = 0; // y
		acadoVariables.y[i * NY + 2] = 0; // z
	}

    // set terminal reference
    acadoVariables.yN[ 0 ] = this->ref.position.x; // x
	acadoVariables.yN[ 1 ] = this->ref.position.y; // y
	acadoVariables.yN[ 2 ] = this->ref.position.z; // z

    for (int i = 0; i < NX; ++i)
	{
		acadoVariables.x0[ i ] = acadoVariables.x[ i ];
	}

    acado_preparationStep();

    // convert quaternion to true roll pitch yaw
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    roll = -roll;

    // convert true RPY to angles (assuming yaw is 0)
    double roll0, pitch0;
    roll0 = -pitch * sin(yaw) + roll * cos(yaw);
    pitch0 = pitch * cos(yaw) + roll * sin(yaw);
    
    // current feedback
    acadoVariables.x0[ 0 ] = msg.pose.pose.position.x;    // x
    acadoVariables.x0[ 1 ] = msg.pose.pose.position.y;    // y
    acadoVariables.x0[ 2 ] = msg.pose.pose.position.z;    // z
    acadoVariables.x0[ 3 ] = msg.twist.twist.linear.x;    // vx
    acadoVariables.x0[ 4 ] = msg.twist.twist.linear.y;    // vy
    acadoVariables.x0[ 5 ] = acadoVariables.x[ 5 ];       // z1
    acadoVariables.x0[ 6 ] = acadoVariables.x[ 6 ];       // z2
    acadoVariables.x0[ 7 ] = roll0;   // psi
    acadoVariables.x0[ 8 ] = pitch0; // theta

    // observer

    // calculate
    int status;
    status = acado_feedbackStep( );

    // publish command
    geometry_msgs::Twist cmd;
    double rolld, pitchd;
    rolld = acadoVariables.u[1] * sin(yaw) + acadoVariables.u[0] * cos(yaw);
    pitchd = acadoVariables.u[1] * cos(yaw) - acadoVariables.u[0] * sin(yaw);
    cmd.linear.x = pitchd / (M_PI / 18) / 2; // pitch
    cmd.linear.y = rolld / (M_PI / 18) / 2; // roll
    cmd.linear.z = acadoVariables.u[2]; // vertical velocity
    ctrl_pub.publish(cmd);

    // shift states
    acado_shiftStates(2, 0, 0);

    // shift control
    acado_shiftControls( 0 );

    // prepare mpc solver
    acado_preparationStep();
}

int main(int argc, char **argv)  
{  
    // initialize the mpc solver
    acado_initializeSolver();

    // initialize state matrix
	for (int i = 0; i < N + 1; ++i)
	{
		acadoVariables.x[i * NX + 0] = 0;
		acadoVariables.x[i * NX + 1] = 0;
		acadoVariables.x[i * NX + 2] = 0;
		acadoVariables.x[i * NX + 3] = 0;
		acadoVariables.x[i * NX + 4] = 0;
		acadoVariables.x[i * NX + 5] = 0;
		acadoVariables.x[i * NX + 6] = 0;
		acadoVariables.x[i * NX + 7] = 0;
		acadoVariables.x[i * NX + 8] = 0;
	}

	//Initiate ROS  
	ros::init(argc, argv, "marker_detector");  
	
    //Create an object of class SubscribeAndPublish that will take care of everything  
	bebop_mpc mpcObject;; 

	ros::spin();  
	return 0;  
} 