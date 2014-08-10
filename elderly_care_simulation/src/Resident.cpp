#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "EventTriggerConstants.h"
#include "PerformTaskConstants.h"
#include "elderly_care_simulation/PerformTask.h"

#include <sstream>
#include "math.h"

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
const double WORLD_POS_X = 0;
const double WORLD_POS_Y = 0;
double px;
double py;
double theta;

// Current task type: -1 corresponds to no task
int currentTaskType = -1;

// Basic health attributes
const int HEALTHY_THRESHOLD = 50;
int happiness = 0;
int amusement = 0;


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

/**
 * Change the resident's state in response to a task that is being performed
 * by a helper.
 * 
 * At this stage, it is assumed for simplicity that the visitor is 
 * consoling and the assistant is entertaining. TODO: In a later release
 * there will be many other types of task. 
 * 
 * @param taskType the type of task that is being performed
 * @return PERFORM_TASK_RESULT_ACCEPTED or PERFORM_TASK_RESULT_FINISHED
 *         (which correspond to 0 and 1 respectively)
 */
int handleTask(int taskType) {
	int result;
	
	switch (taskType) {
		case EVENT_TRIGGER_EVENT_TYPE_VISITOR:
			// The visitor is consoling us
			happiness += 1;
			if (happiness > HEALTHY_THRESHOLD) {
				ROS_INFO("Happiness raised to %d and I'm now happy as can be!", happiness);
				result = PERFORM_TASK_RESULT_FINISHED;
				currentTaskType = NO_CURRENT_TASK;
			} else {
				ROS_INFO("Happiness raised to %d, but I could still do with some more consoling...", happiness);
				result = PERFORM_TASK_RESULT_ACCEPTED;
			}
			break;
		case EVENT_TRIGGER_EVENT_TYPE_ASSISTANT:
			// The assistant is entertaining us
			amusement += 1;
			if (amusement > HEALTHY_THRESHOLD) {
				ROS_INFO("Amusement raised to %d and I've had enough!", amusement);
				result = PERFORM_TASK_RESULT_FINISHED;
				currentTaskType = NO_CURRENT_TASK;
			} else {
				ROS_INFO("Amusement raised to %d, keep being funny.", amusement);
				result = PERFORM_TASK_RESULT_ACCEPTED;
			}
		    break;
	}
	
	return result;
}

/**
 * Handler for PerformTask.srv service 
 * 
 * Robots and Visitors use this to perform tasks on the resident.
 * 
 * In the request there will be an event type. The resident will check
 * if this corresponds to the type of event they are currently accepting.
 * 
 * Based on this the response will contain one of the following codes:
 *   0 - I accepted the task and updated my state
 *   1 - I accepted the task and you can now stop
 *   2 - I am busy at the moment, try again
 */
bool performTaskServiceHandler(elderly_care_simulation::PerformTask::Request &req,
				   elderly_care_simulation::PerformTask::Response &res) {
	if (currentTaskType == -1) {
		// I don't yet have a task, make this one our current task
		currentTaskType = req.taskType;
	}
	
	if (req.taskType == currentTaskType) {
		// We must be dealing with the current helper
		res.result = handleTask(req.taskType);
	} else {
		// We are busy with another task
		res.result = PERFORM_TASK_RESULT_BUSY;
	}
		
	return true;
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
ros::init(argc, argv, "Resident");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//advertise() function will tell ROS that you want to publish on a given topic_
//to stage
ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

//subscribe to listen to messages coming from stage
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);

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
