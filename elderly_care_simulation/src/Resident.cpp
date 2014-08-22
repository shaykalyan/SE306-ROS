#include "ros/ros.h"
#include <sstream>
#include <unistd.h>

#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Empty.h"
#include <geometry_msgs/Point.h>

#include <queue>
#include <tf/tf.h>

#include "PerformTaskConstants.h"
#include "elderly_care_simulation/PerformTask.h"
#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"
#include "EventTriggerUtility.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/FindPath.h"

#include "Robot.h"
#include "Resident.h"
#include "Poi.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

// Current task type: -1 corresponds to no task

;Resident::Resident() {
    currentTaskType = -1;

    // How long the Resident is to receive friendship for
    FRIENDSHIP_DURATION = 100;
    friendshipCount = 0;

    // Basic health attributes
    HEALTHY_THRESHOLD = 50;
    happiness = 0;
    amusement = 0;
   
}

Resident::~Resident() {
    
}

/**
 * Method to perform after a task has been performed on this resident.
 * Moves the robot to the left and right to acknowledge his task has been performed.
 */
void Resident::taskCompleted(const std_msgs::Empty empty){
	/*geometry_msgs::Point locationOne;
    locationOne.x = currentLocation.position.x + 1;
    locationOne.y = currentLocation.position.y;

    geometry_msgs::Point locationTwo;
    locationTwo.x = currentLocation.position.x - 1;
    locationTwo.y = currentLocation.position.y;

    geometry_msgs::Point locationThree;
    locationThree.x = currentLocation.position.x;
    locationThree.y = currentLocation.position.y;

    locationQueue.push(locationOne);
    locationQueue.push(locationTwo);
    locationQueue.push(locationThree);*/
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
int Resident::handleTask(int taskType) {
	int result;
	
	switch (taskType) {
        case EVENT_TRIGGER_EVENT_TYPE_FRIEND_RELATIVE:

            // The friend is interacting with me
            if (++friendshipCount > FRIENDSHIP_DURATION) {
                ROS_INFO("Resident: Thanks for the friendship. You can go now!");
                result = PERFORM_TASK_RESULT_FINISHED;
                currentTaskType = NO_CURRENT_TASK;

				std_msgs::Empty emptyMessage;
				taskCompleted(emptyMessage);
                friendshipCount = 0;                
            }
            else {
                result = PERFORM_TASK_RESULT_ACCEPTED;
            }
            break;

		case EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT:
			// The visitor is giving me moral support
			happiness += 1;
			if (happiness > HEALTHY_THRESHOLD) {
				ROS_INFO("Resident: Happiness raised to %d and I'm now happy enough!", happiness);
				result = PERFORM_TASK_RESULT_FINISHED;
				currentTaskType = NO_CURRENT_TASK;

				std_msgs::Empty emptyMessage;
				taskCompleted(emptyMessage);
			} else {
				ROS_INFO("Resident: Happiness raised to %d, continue consoling", happiness);
				result = PERFORM_TASK_RESULT_ACCEPTED;
			}
			break;

		case EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT:
			// The assistant is entertaining me
			amusement += 1;
			if (amusement > HEALTHY_THRESHOLD) {
				ROS_INFO("Resident: Amusement raised to %d and I've had enough!", amusement);
				result = PERFORM_TASK_RESULT_FINISHED;
				currentTaskType = NO_CURRENT_TASK;

				std_msgs::Empty emptyMessage;
				taskCompleted(emptyMessage);
			} else {
				ROS_INFO("Resident: Amusement raised to %d, keep being funny.", amusement);
				result = PERFORM_TASK_RESULT_ACCEPTED;
			}
		    break;
        default:
            result = -1;
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
bool Resident::performTaskServiceHandler(elderly_care_simulation::PerformTask::Request &req,
				   elderly_care_simulation::PerformTask::Response &res) {
					   
	//ROS_INFO("Received service call with task type: %d", req.taskType);
	
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

void Resident::diceTriggerCallback(elderly_care_simulation::DiceRollTrigger msg){
    elderly_care_simulation::EventTrigger msgOut;
    msgOut.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msgOut.result = EVENT_TRIGGER_RESULT_UNDEFINED;

    // Modify this switch statement to control random events
    switch(msg.type) {
        case MORAL_SUPPORT:
            ROS_INFO("Resident: I want moral support");
            msgOut.event_type = EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT;
            msgOut.event_priority = EVENT_TRIGGER_PRIORITY_MEDIUM;
            break;
        case ILL:
            ROS_INFO("Resident: I am ill");
            // TODO:
            return;
            break;
        case VERY_ILL:
            ROS_INFO("Resident: I am very ill");
            // TODO:
            return;
            break;
        default:
            ROS_INFO("Resident: Unknown.");
            return;
    }
    msgOut.event_weight = getEventWeight(msgOut.event_type);
    ROS_INFO("Resident: Sending request to scheduler");
    residentEventPub.publish(msgOut);
}

Resident resident;

void callStage0domCallback(const nav_msgs::Odometry msg) {
    resident.stage0domCallback(msg);
}
void callDiceTriggerCallback(elderly_care_simulation::DiceRollTrigger msg){
    resident.diceTriggerCallback(msg);

}

void callUpdateDesiredLocationCallback(const geometry_msgs::Point location){
    resident.updateDesiredLocationCallback(location);
}

void callTaskCompleted(const std_msgs::Empty empty){
    resident.taskCompleted(empty);
}

bool callPerformTaskServiceHandler(elderly_care_simulation::PerformTask::Request &req,
                   elderly_care_simulation::PerformTask::Response &res){
    return resident.performTaskServiceHandler(req, res);
}
int main(int argc, char **argv) {

    // ROS initialiser calls
    ros::init(argc, argv, "Resident");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(25);

    resident = Resident();

      // Initialise publishers
    resident.robotNodeStagePub = nodeHandle.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 
    resident.residentEventPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("resident_event",1000, true);

    // Initialise subscribers
    resident.stageOdoSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth", 1000, callStage0domCallback);
    resident.diceTriggerSub = nodeHandle.subscribe<elderly_care_simulation::DiceRollTrigger>("dice_roll_trigger", 1000, callDiceTriggerCallback);

    resident.locationInstructionsSub = nodeHandle.subscribe<geometry_msgs::Point>("robot_0/location", 1000, callUpdateDesiredLocationCallback);

    resident.pathOfResidentSub = nodeHandle.subscribe<std_msgs::Empty>("robot_0/resident_respond", 1000, callTaskCompleted);

    resident.pathFinderService = nodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path"); 

    // Advertise that the Resident responds to PerformTask service calls
	ros::ServiceServer service = nodeHandle.advertiseService("perform_task", callPerformTaskServiceHandler);

	// A count of howmany messages we have sent
	int count = 0;

	while (ros::ok())
	{
		resident.updateCurrentVelocity();

		// Every 5 seconds ...
		if (count % 50 == 0) {
			
            // ... decrease health values
			
			resident.amusement = (resident.amusement - 15) > 0 ? resident.amusement - 15 : 0;
			ROS_INFO("Amusement level fell to %d", resident.amusement);

			resident.happiness = (resident.happiness - 10) > 0 ? resident.happiness - 10 : 0;
			ROS_INFO("Happiness level fell to %d", resident.happiness);
		}

		resident.robotNodeStagePub.publish(resident.currentVelocity);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

    return 0;

}
