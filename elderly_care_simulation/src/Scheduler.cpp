#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "elderly_care_simulation/task_request.h"

#include <sstream>
#include "math.h"
#include <time.h>

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
const double WORLD_POS_X = -3;
const double WORLD_POS_Y = 0;
double px;
double py;
double theta;

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = WORLD_POS_X + msg.pose.pose.position.x;
	py = WORLD_POS_Y + msg.pose.pose.position.y;
	ROS_INFO("Current x position is: %f", px);
	ROS_INFO("Current y position is: %f", py);
}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

int main(int argc, char **argv)
{

	 //initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = M_PI/2.0;
	px = WORLD_POS_X;
	py = WORLD_POS_Y;
	
	//Initial velocity
	linear_x = 0.0;
	angular_z = 1.0;
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Scheduler");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000); 
	// ros::Publisher task_request_pub = n.advertise<elderly_care_simulation::tr>("task_request",1000);
	ros::ServiceClient task_request_client = n.serviceClient<elderly_care_simulation::task_request>("task_request");

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_1/odom",1000, StageOdom_callback);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_1/base_scan",1000, StageLaser_callback);

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;
	time_t time1, time2;
	time(&time1);
	unsigned int id = 0;

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;

	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
	        
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		
		if(difftime(time(&time2), time1) > 3){
			ROS_WARN("Sending.....");
			elderly_care_simulation::task_request srv;
			srv.request.id = id;
			srv.request.type = rand()%100;

			if(task_request_client.call(srv) == 1){
				ROS_WARN("Successfully Sent. %d", (unsigned int)srv.response.result);
			}else {
				ROS_ERROR("Dog Shit Happened.");
			}
			time(&time1); // Reset Time

			id++;
		}

		
		// Initialise task req
		// std::stringstream ss;
		// std_msgs::String msg;
		// ss << "Scheduler: Publish to Task Request";
		// msg.data = ss.str();
		// ROS_INFO("%s", msg.data.c_str());
		
		// elderly_care_simulation::tr s_tr;
		// s_tr.id = 2;
		// s_tr.task = 20;
		// task_request_pub.publish(s_tr); // Publish it
		// ROS_INFO("ID: %d, TASK: %d", s_tr.id, s_tr.task);
		
		//--------------------------------------------------------
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;

}
