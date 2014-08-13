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

#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"
#include "EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <unistd.h> // sleep

// Velocity
double linearX;
double angularZ;

// Pose
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

// Signatures
ros::Publisher robotNodeStagePub;
ros::Subscriber stageOdoPub;
ros::Subscriber diceTriggerSub;
ros::Publisher residentEvent_pub;
void stageOdomCallback(nav_msgs::Odometry msg);
void diceTriggerCallback();

/**
    Process odometry messages from Stage
*/
void stageOdomCallback(nav_msgs::Odometry msg) {
	
	px = WORLD_POS_X + msg.pose.pose.position.x;
	py = WORLD_POS_Y + msg.pose.pose.position.y;
	// ROS_INFO("Current x position is: %f", px);
	// ROS_INFO("Current y position is: %f", py);
}

void diceTriggerCallback(elderly_care_simulation::DiceRollTrigger msg) {
    elderly_care_simulation::EventTrigger msg_out;
    msg_out.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg_out.result = EVENT_TRIGGER_RESULT_FAILURE;

    switch(msg.type) {
        case MORAL_SUPPORT:
            ROS_INFO("I really need moral support right now ...");
            msg_out.event_type = EVENT_TRIGGER_EVENT_TYPE_VISITOR;
            break;
        case ENTERTAINMENT:
			ROS_INFO("I really need some entertainment ...");
			msg_out.event_type = EVENT_TRIGGER_EVENT_TYPE_ASSISTANT;
			break;
    }

    ROS_INFO("Sending request to scheduler");
    residentEvent_pub.publish(msg_out);
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
					   
	ROS_INFO("Received service call with task type: %d", req.taskType);
	
	if (currentTaskType == NO_CURRENT_TASK) {
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

/**
    Process 
*/

int main(int argc, char **argv) {

    // ROS initialiser calls
    ros::init(argc, argv, "Resident");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    // Initialise pose (must be same as world file)
	theta = M_PI/2.0;
	px = WORLD_POS_X;
	py = WORLD_POS_Y;
	
	// Initialise velocity
	linearX = 0.0;
	angularZ = 0.0;
	
    // Initialise publishers
    robotNodeStagePub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 
    residentEvent_pub = n.advertise<elderly_care_simulation::EventTrigger>("resident_event",1000, true);

    // Initialise subscribers
    stageOdoPub = n.subscribe<nav_msgs::Odometry>("robot_0/odom", 1000, stageOdomCallback);
    diceTriggerSub = n.subscribe<elderly_care_simulation::DiceRollTrigger>("dice_roll_trigger", 1000, diceTriggerCallback);

    // Initialise messages
    geometry_msgs::Twist robotNodeCmdvel;
    
    // Advertise that the Resident responds to PerformTask service calls
	ros::ServiceServer service = n.advertiseService("perform_task", performTaskServiceHandler);

	//a count of howmany messages we have sent
	int count = 0;

	while (ros::ok())
	{
		// Publish to Stage
		robotNodeCmdvel.linear.x = linearX;
		robotNodeCmdvel.angular.z = angularZ;
		robotNodeStagePub.publish(robotNodeCmdvel);
		
		// Every once in a while, decrease health values
		if (count % 50 == 0) {
			// Every 5 secs
			
			amusement = (amusement - 15) > 0 ? amusement - 15 : 0;
			ROS_INFO("Amusement level fell to %d", amusement);

			happiness = (happiness - 10) > 0 ? happiness - 10 : 0;
			ROS_INFO("Happiness level fell to %d", happiness);
		}
		
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

    return 0;

}
