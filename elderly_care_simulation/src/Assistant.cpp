#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"
#include "EventTriggerConstants.h"
#include "PerformTaskConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/PerformTask.h"
#include <unistd.h>

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
const double WORLD_POS_X = -3;
const double WORLD_POS_Y = 0;
double px;
double py;
double theta;

// Tasks
const int MY_TASK = EVENT_TRIGGER_EVENT_TYPE_ASSISTANT;
bool performingTask = false;

// Topics
ros::Publisher RobotNode_stage_pub;
ros::Publisher EventTrigger_pub;
ros::Subscriber EventTrigger_sub;

// Services
ros::ServiceClient performTaskClient;

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
	// create response message
	elderly_care_simulation::EventTrigger msg;
	msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;
	msg.event_type = EVENT_TRIGGER_EVENT_TYPE_ASSISTANT;
	msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

	EventTrigger_pub.publish(msg);
	ROS_INFO("Assistant Reply Message Sent");
}

/**
 * Send a message to Stage to start rotation of this robot.
 */
void startRotating() {
	geometry_msgs::Twist RobotNode_cmdvel;
	RobotNode_cmdvel.linear.x = linear_x;
	RobotNode_cmdvel.angular.z = 2.0;
	RobotNode_stage_pub.publish(RobotNode_cmdvel);
}

/**
 * Send a message to Stage to stop rotation of this robot.
 */
void stopRotating() {
	geometry_msgs::Twist RobotNode_cmdvel;
	RobotNode_cmdvel.linear.x = linear_x;
	RobotNode_cmdvel.angular.z = 0.0;
	RobotNode_stage_pub.publish(RobotNode_cmdvel);
}

void EventTrigger_callback(elderly_care_simulation::EventTrigger msg)
{
	if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {
		if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_ASSISTANT) {
			ROS_INFO("Assistant Message Recieved");

			performingTask = true;
		}
	}
}

/**
 * Perform a task on the resident by making a service call to them.
 */
void performTask() {
	
	// Generate the service call
	elderly_care_simulation::PerformTask performTaskSrv;
	performTaskSrv.request.taskType = MY_TASK;
	
	// Make the call using the client
	if (!performTaskClient.call(performTaskSrv)) {
		throw std::runtime_error("Service call to the initiate task with Resident failed");
	}
	
	switch (performTaskSrv.response.result) {
		case PERFORM_TASK_RESULT_ACCEPTED:
			// Resident has accepted the task but keep going
			ROS_INFO("Resident has accepted the task but says keep going");
			startRotating();
			break;
		case PERFORM_TASK_RESULT_FINISHED:
			// Resident accepted the task and has had enough
			ROS_INFO("Resident has accepted the task and has had enough");
			performingTask = false;
			stopRotating();
			EventTrigger_reply();
			break;
		case PERFORM_TASK_RESULT_BUSY:
			// Resident is busy
			ROS_INFO("Resident is busy");
			break;
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
	ros::init(argc, argv, "Assistant");

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
	
	// Create a client to make service requests to the Resident
	performTaskClient = n.serviceClient<elderly_care_simulation::PerformTask>("perform_task");

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
		//RobotNode_stage_pub.publish(RobotNode_cmdvel);
		
		ros::spinOnce();
		
		if (performingTask) {
			performTask();
		}

		loop_rate.sleep();
		++count;
	}

	return 0;
}
