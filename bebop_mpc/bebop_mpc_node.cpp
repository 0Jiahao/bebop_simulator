#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <time.h>

#define NX          ACADO_NX	/* number of differential states */
#define NXA         ACADO_NXA	/* number of alg. states */
#define NU          ACADO_NU	/* number of control inputs */
#define NOD         ACADO_NOD  /* Number of online data values. */
#define N          	ACADO_N		/* number of control intervals */
#define NY			ACADO_NY	/* number of references, nodes 0..N - 1 */
#define NYN			ACADO_NYN
#define M_PI 3.14159265358979323846

#define SHOW_PATH   true

using namespace std;
using namespace Eigen;

// mpc variables
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


const int wps_n = 2;
double wps_x[wps_n] = {-3.0, 3.0};
double wps_y[wps_n] = { 0.0, 0.0};
double wps_z[wps_n] = { 1.5, 1.5};
int wps_idx = 0;

class bebop_mpc
{
    public:
        bebop_mpc();
    private:
        // member
        ros::NodeHandle nh;   		            // define node
#if SHOW_PATH
        ros::Publisher path_pub;
        ros::Publisher obs_pub;
        // start time
	    ros::Time startTime;
#endif
		ros::Subscriber odom_sub;               // mav state subscriber 
		ros::Publisher  ctrl_pub;               // control command publisher
        geometry_msgs::Pose ref;                // reference pose
        void solve(const nav_msgs::Odometry& msg);
};

bebop_mpc::bebop_mpc()
{
#if SHOW_PATH
    this->path_pub = nh.advertise<nav_msgs::Path>("/rviz/path",1);
    this->obs_pub = nh.advertise<visualization_msgs::MarkerArray>("/rviz/obs",1);
    startTime = ros::Time::now();
#endif
    this->odom_sub = nh.subscribe("/bebop2/odometry", 1, &bebop_mpc::solve,this);
    this->ctrl_pub = nh.advertise<geometry_msgs::Twist>("/bebop2_auto/cmd_vel",1);
    this->ref.position.x = wps_x[wps_idx];
    this->ref.position.y = wps_y[wps_idx];
    this->ref.position.z = wps_z[wps_idx];
    // this->ref.orientation.z = M_PI; //reference yaw
}

void bebop_mpc::solve(const nav_msgs::Odometry& msg)
{
    // start clock
    clock_t t_start = clock();

    // convert quaternion to true roll pitch yaw
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // online data
    double Ts = 1 / 15;
    ros::Duration diffTime = ros::Time::now() - startTime;
	double dt = diffTime.toSec();
    double obs_x = 0;
    double obs_y = 0; //1 * sin(0.5 * dt);
    double obs_vx = 0;
    double obs_vy = 0;//0.5 * cos(0.5 * dt);
#if SHOW_PATH
    visualization_msgs::MarkerArray mks;
#endif
    for (int i = 0; i < N; i++)
	{
        acadoVariables.od[ i * NOD + 0 ] = 0;
        acadoVariables.od[ i * NOD + 1 ] = obs_y + i * Ts * obs_vy;
        
#if SHOW_PATH
        visualization_msgs::Marker mk;
        mk.header.stamp = ros::Time::now();
        mk.header.frame_id = "world";
        mk.ns = "obs";
        mk.id = i;
        mk.type = visualization_msgs::Marker::CYLINDER;
		mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position.x = acadoVariables.od[ i * NOD + 0 ];
        mk.pose.position.y = acadoVariables.od[ i * NOD + 1 ];
        mk.pose.position.z = 1.5;
        mk.scale.x = 0.5;
        mk.scale.y = 0.5;
        mk.scale.z = 3;
        mk.lifetime = ros::Duration(1);
        mk.color.r = 1.0;
        mk.color.g = 1.0;
        mk.color.b = 0.0;
        mk.color.a = 1;
        mks.markers.push_back(mk);
#endif
	}

#if SHOW_PATH
    obs_pub.publish(mks);
#endif

    // set reference
    for (int i = 0; i < N; ++i)
	{
		acadoVariables.y[ i * NY + 0 ] = 0;
        acadoVariables.y[ i * NY + 1 ] = 0; 
        acadoVariables.y[ i * NY + 2 ] = 0; 
        acadoVariables.y[ i * NY + 3 ] = 0;
        acadoVariables.y[ i * NY + 4 ] = 0; 
	}

    // set terminal reference
    acadoVariables.yN[ 0 ] = this->ref.position.x; // x
	acadoVariables.yN[ 1 ] = this->ref.position.y; // y
	acadoVariables.yN[ 2 ] = this->ref.position.z; // z
    acadoVariables.yN[ 3 ] = 0; // y
	acadoVariables.yN[ 4 ] = 0; // z
    acadoVariables.yN[ 5 ] = 0; // z

    for (int i = 0; i < NX; ++i)
	{
		acadoVariables.x0[ i ] = acadoVariables.x[ i ];
	}

    // cout << acadoVariables.od[0] << endl;

    acado_preparationStep();
    
    // convert true RPY to angles (assuming yaw is 0)
    double roll0, pitch0;
    roll0 = -pitch * sin(yaw) + roll * cos(yaw);
    pitch0 = pitch * cos(yaw) + roll * sin(yaw);
    // cout << "roll0: " << roll0 << " pitch0: " << pitch0 << " yaw: " << yaw << endl;

// SWITCH WAYPOINT
    if(sqrt(pow(msg.pose.pose.position.x - this->ref.position.x,2) + pow(msg.pose.pose.position.y - this->ref.position.y,2) + pow(msg.pose.pose.position.z - this->ref.position.z,2)) < 0.2)
    {
        wps_idx++;
        this->ref.position.x = wps_x[wps_idx % wps_n];
        this->ref.position.y = wps_y[wps_idx % wps_n];
        this->ref.position.z = wps_z[wps_idx % wps_n];
    }
// SWITCH WAYPOINT 

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
    acadoVariables.x0[ 9 ] = yaw; // yaw
    acadoVariables.x0[ 10] = acadoVariables.x[ 10];
    acadoVariables.x0[ 11] = acadoVariables.x[ 11];

    // calculate
    int status;
    status = acado_feedbackStep( );
    ROS_INFO_STREAM("SOLVER STATUS: " << status);

    cout << acadoVariables.u[4] << endl;

    // publish command
    if(status == 0)
    {
        geometry_msgs::Twist cmd;
        double rolld, pitchd;
        rolld =  acadoVariables.u[1] * sin(yaw) + acadoVariables.u[0] * cos(yaw);
        pitchd = acadoVariables.u[1] * cos(yaw) - acadoVariables.u[0] * sin(yaw);
        cmd.linear.x = pitchd / (M_PI / 18); // pitch
        cmd.linear.y = -rolld / (M_PI / 18); // roll(positive move to left in bebop autonomy)
        cmd.linear.z = acadoVariables.u[2]; // vertical velocity
        cmd.angular.z = acadoVariables.u[3] / (M_PI / 2); // yawrate
        
        ctrl_pub.publish(cmd);

#if SHOW_PATH
        nav_msgs::Path path;
        path.header.frame_id = "world";
        for(int i = 0; i < N; i++)
        {
            geometry_msgs::PoseStamped prediction;
            prediction.pose.position.x = acadoVariables.x[i * NX + 0];
            prediction.pose.position.y = acadoVariables.x[i * NX + 1];
            prediction.pose.position.z = acadoVariables.x[i * NX + 2];
            path.poses.push_back(prediction);
        }

        path_pub.publish(path);
#endif
    }

    // shift states
    acado_shiftStates(0, 0, 0);

    // shift control
    acado_shiftControls( 0 );

    // prepare mpc solver
    acado_preparationStep();

    // end clock
    clock_t t_end = clock();
	cout << "[INFO] solving time: " << ((float)(t_end - t_start))/CLOCKS_PER_SEC << endl;
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