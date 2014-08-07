#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"
#include "EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <unistd.h>

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
const double WORLD_POS_X = 3;
const double WORLD_POS_Y = 0;
double px;
double py;
double theta;

ros::Publisher RobotNode_stage_pub;
ros::Publisher EventTrigger_pub;
ros::Subscriber EventTrigger_sub;

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = WORLD_POS_X + msg.pose.pose.position.x;
	py = WORLD_POS_Y + msg.pose.pose.position.y;
}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

void EventTrigger_reply() {
	elderly_care_simulation::EventTrigger msg;
	msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;
	msg.event_type = EVENT_TRIGGER_EVENT_TYPE_VISITOR;
	msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

	EventTrigger_pub.publish(msg);
	ROS_INFO("Visitor Reply Message Sent");
}

void EventTrigger_callback(elderly_care_simulation::EventTrigger msg)
{
	if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {
		if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_VISITOR) {
			ROS_INFO("Visitor Message Recieved");

			// carry out activity
			// update angular z and inform stage
			geometry_msgs::Twist RobotNode_cmdvel;
			RobotNode_cmdvel.linear.x = linear_x;
			RobotNode_cmdvel.angular.z = 2.0;
			RobotNode_stage_pub.publish(RobotNode_cmdvel);

			// stall for 5 seconds to allow robot to spin
			sleep(4);

			// reset angular z and update stage
			RobotNode_cmdvel.linear.x = linear_x;
			RobotNode_cmdvel.angular.z = 0.0;
			RobotNode_stage_pub.publish(RobotNode_cmdvel);
			// reply done function
			EventTrigger_reply();
		}
	}
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
	angular_z = 0.0;
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Visitor");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_2/cmd_vel",1000);
	EventTrigger_pub = n.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_2/odom",1000, StageOdom_callback);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_2/base_scan",1000,StageLaser_callback);
	EventTrigger_sub = n.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, EventTrigger_callback);


	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

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
		
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;

}
