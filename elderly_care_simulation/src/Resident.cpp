#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "EventTriggerConstants.h"
#include "PerformTaskConstants.h"
#include "elderly_care_simulation/PerformTask.h"
#include <queue>
#include <tf/tf.h>
#include "std_msgs/Empty.h"

#include <sstream>
#include "math.h"

#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"
#include "EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <unistd.h> // sleep

// Current task type: -1 corresponds to no task
int currentTaskType = -1;

// Basic health attributes
const int HEALTHY_THRESHOLD = 50;
int happiness = 0;
int amusement = 0;

// Signatures
ros::Publisher robotNodeStagePub;
ros::Subscriber stageOdoSub;
ros::Subscriber diceTriggerSub;
ros::Publisher residentEventPub;
ros::Subscriber locationInstructionsSub;
ros::Subscriber pathOfResidentSub;

double currentAngle;

// Current velocity of the Robot
geometry_msgs::Twist currentVelocity;

// Current location of the robot
geometry_msgs::Pose currentLocation;

// Locations to visit
std::queue<geometry_msgs::Point> locationQueue;

void stageOdomCallback(nav_msgs::Odometry msg);
void diceTriggerCallback();

void taskGetPerformed(const std_msgs::Empty){
	geometry_msgs::Point locationOne;
    locationOne.x = 0;
    locationOne.y = -2;

    geometry_msgs::Point locationTwo;
    locationTwo.x = 0;
    locationTwo.y = 2;

    geometry_msgs::Point locationThree;
    locationThree.x = 0;
    locationThree.y = 0;

    locationQueue.push(locationOne);
    locationQueue.push(locationTwo);
    locationQueue.push(locationThree);
}

/**
    Process odometry messages from Stage
*/
void stageOdomCallback(const nav_msgs::Odometry msg) {
	
	 //Update Current Position
    currentLocation = msg.pose.pose;
    double x = currentLocation.orientation.x;
    double y = currentLocation.orientation.y;
    double z = currentLocation.orientation.z;
    double w = currentLocation.orientation.w;
    ROS_INFO("THE LOCATION IS: %f, %f", currentLocation.position.x, currentLocation.position.y);
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
    currentAngle = yaw;

}

void updateDesiredLocationCallback(const geometry_msgs::Point location)
{   
    // Add location to the locationQueue queue
    locationQueue.push(location);
}

bool doubleEquals(double a, double b, double difference)
{
    return std::abs(a - b) < difference;
}

double normalizeAngle(double angle)
{
    while (angle < 0) {
        angle += 2 * M_PI;
    }
    while (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}

bool turnAnticlockwise(double currentAngle, double desiredAngle)
{   
    if (currentAngle < 0) {
        currentAngle = 2 * M_PI + currentAngle;
    }
    if (desiredAngle < 0) {
        desiredAngle = 2 * M_PI + desiredAngle;
    }
    desiredAngle = normalizeAngle(desiredAngle - currentAngle);
    return desiredAngle < M_PI;
    
}

bool atDesiredLocation()
{  
    if (locationQueue.empty()) {
        return true;
    } else {
        double toleratedDifference = 0.15;
        geometry_msgs::Point desiredLocation = locationQueue.front();

        if( doubleEquals(currentLocation.position.x, desiredLocation.x, toleratedDifference) &&
            doubleEquals(currentLocation.position.y, desiredLocation.y, toleratedDifference)) {
            locationQueue.pop();
            return atDesiredLocation();
        }
    }
    return false;
      
}

void updateCurrentVelocity()
{
    if (atDesiredLocation()) {
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = 0;
    }
    // Find the correct angle
    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation

    geometry_msgs::Point desiredLocation = locationQueue.front();

    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;
    
    // Thank god we're only doing 2D stuff
    double desiredAngle = atan2(directionVector.y, directionVector.x);

    if (! doubleEquals(currentAngle, desiredAngle, 0.15)) {
        // Turn towards angle
        currentVelocity.linear.x = 0;
        
        if (turnAnticlockwise(currentAngle, desiredAngle)) {
            // Turn anti clockwise
            currentVelocity.angular.z = 1;
        } else {
            // Turn clockwise
            currentVelocity.angular.z = -1;
        }
    } else {
        // Go forward
        currentVelocity.linear.x = 1;
        currentVelocity.angular.z = 0;
    }
}


void diceTriggerCallback(elderly_care_simulation::DiceRollTrigger msg) {
    elderly_care_simulation::EventTrigger msgOut;
    msgOut.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msgOut.result = EVENT_TRIGGER_RESULT_FAILURE;

    switch(msg.type) {
        case MORAL_SUPPORT:
            ROS_INFO("I really need moral support right now ...");
            msgOut.event_type = EVENT_TRIGGER_EVENT_TYPE_VISITOR;
            break;
        case ENTERTAINMENT:
			ROS_INFO("I really need some entertainment ...");
			msgOut.event_type = EVENT_TRIGGER_EVENT_TYPE_ASSISTANT;
			break;
    }

    ROS_INFO("Sending request to scheduler");
    residentEventPub.publish(msgOut);
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

				std_msgs::Empty emptyMessage;
				taskGetPerformed(emptyMessage);
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

				std_msgs::Empty emptyMessage;
				taskGetPerformed(emptyMessage);
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
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);
	
    // Initialise publishers
    robotNodeStagePub = nodeHandle.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 
    residentEventPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("resident_event",1000, true);

    // Initialise subscribers
    stageOdoSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth", 1000, stageOdomCallback);
    diceTriggerSub = nodeHandle.subscribe<elderly_care_simulation::DiceRollTrigger>("dice_roll_trigger", 1000, diceTriggerCallback);

    locationInstructionsSub = nodeHandle.subscribe<geometry_msgs::Point>("robot_0/location", 1000, updateDesiredLocationCallback);

    pathOfResidentSub = nodeHandle.subscribe<std_msgs::Empty>("robot_0/resident_respond", 1000, taskGetPerformed);
    // Initialise messages
    geometry_msgs::Twist robotNodeCmdvel;
    
    // Advertise that the Resident responds to PerformTask service calls
	ros::ServiceServer service = nodeHandle.advertiseService("perform_task", performTaskServiceHandler);

	//a count of howmany messages we have sent
	int count = 0;

	while (ros::ok())
	{
		updateCurrentVelocity();

		// Every once in a while, decrease health values
		if (count % 50 == 0) {
			// Every 5 secs
			
			amusement = (amusement - 15) > 0 ? amusement - 15 : 0;
			ROS_INFO("Amusement level fell to %d", amusement);

			happiness = (happiness - 10) > 0 ? happiness - 10 : 0;
			ROS_INFO("Happiness level fell to %d", happiness);
		}

		robotNodeStagePub.publish(currentVelocity);
		
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

    return 0;

}
