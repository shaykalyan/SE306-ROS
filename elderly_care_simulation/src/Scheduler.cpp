#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"
#include "EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <time.h>

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
const double WORLD_POS_X = 10;
const double WORLD_POS_Y = 10;
double px;
double py;
double theta;

// 0 = VISITOR // 1 = ASSISTANT
bool robot_switch = true;
bool readyToSend = true;

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = WORLD_POS_X + msg.pose.pose.position.x;
	py = WORLD_POS_Y + msg.pose.pose.position.y;
	// ROS_INFO("Current x position is: %f", px);
	// ROS_INFO("Current y position is: %f", py);
}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

void EventTrigger_callback(elderly_care_simulation::EventTrigger msg)
{
	
	if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_RESPONSE) {
		ROS_INFO("Recieved a Response");
		if(msg.result == EVENT_TRIGGER_RESULT_SUCCESS){
			ROS_INFO("Recieved from %d", msg.event_type);

			// reset ability to send
			readyToSend = true;
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
ros::init(argc, argv, "Scheduler");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//advertise() function will tell ROS that you want to publish on a given topic_
//to stage
ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000);
ros::Publisher EventTrigger_pub = n.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);

//subscribe to listen to messages coming from stage
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_1/odom",1000, StageOdom_callback);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_1/base_scan",1000,StageLaser_callback);
ros::Subscriber EventTrigger_sub = n.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, EventTrigger_callback);


ros::Rate loop_rate(10);

//a count of howmany messages we have sent
int count = 0;
int tick = 0;

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

	if (readyToSend) {
		// block scheduler
		readyToSend = false;	

		// create message
		elderly_care_simulation::EventTrigger msg;
		msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
		msg.result = EVENT_TRIGGER_RESULT_FAILURE;
		// msg.event_type = EVENT_TRIGGER_EVENT_TYPE_VISITOR; 

		if (robot_switch) { 	// true = request visitor
			msg.event_type = EVENT_TRIGGER_EVENT_TYPE_VISITOR; 
		} else {				// false = request assistant
			msg.event_type = EVENT_TRIGGER_EVENT_TYPE_ASSISTANT;
		}

		ROS_INFO("Publising to %d", msg.event_type);
		EventTrigger_pub.publish(msg);

		
		robot_switch = !robot_switch;
	}

	ros::spinOnce();

	loop_rate.sleep();
	++count;
}

return 0;

}
