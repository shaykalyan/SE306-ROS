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

;Resident::Resident(){
    currentTaskType = EVENT_TRIGGER_EVENT_TYPE_UNDEFINED;

    taskProgress[EVENT_TRIGGER_EVENT_TYPE_EAT] = 0;
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_SHOWER] = 0;
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_EXERCISE] = 0;
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_CONVERSATION] = 0;
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT] = 0;    
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_RELATIVE] = 0;
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_FRIEND] = 0;
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_ILL] = 0;
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_VERY_ILL] = 0;
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_MEDICATION] = 0;
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_COOK] = 0;
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT] = 0;
    taskProgress[EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP] = 0;

    navigatingToPoiForTask = false;
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

void Resident::resetTaskProgress(int taskType) {
    taskProgress[taskType] = 0;
}

/**
 * Reset the current task to UNDEFINED and clear all task progress states.
 */
void Resident::clearAllTasks() {
    currentTaskType = EVENT_TRIGGER_RESULT_UNDEFINED;

    for(std::map<int, int >::iterator iter = taskProgress.begin(); iter != taskProgress.end(); ++iter) {
        taskProgress[iter->first] = 0;
    }
}

/**
 * Update a task's progress in response to a task that is being performed
 * by a helper.
 * 
 * @param taskType the type of task that is being performed
 * @return PERFORM_TASK_RESULT_ACCEPTED or PERFORM_TASK_RESULT_FINISHED
 *         (which correspond to 0 and 1 respectively)
 */
int Resident::handleTask(int taskType) {
    int result = -1;
    std::string taskName = eventTypeToString(taskType);

    // Check that a valid task type has been given
    if (taskProgress.count(taskType) == 0) {
        ROS_ERROR("Unknown event type: %d", taskType);
        throw std::runtime_error("Unknown event type");
    }

    int progress = taskProgress[taskType];
    progress += 1;
    taskProgress[taskType] = progress;

    if (progress >= TASK_PROGRESS_THRESHOLD) {
        ROS_INFO("Resident: Helper can finish performing the %s task.",  taskName.c_str());
        result = PERFORM_TASK_RESULT_FINISHED;

        // Reset task progress
        currentTaskType = EVENT_TRIGGER_EVENT_TYPE_UNDEFINED;
        resetTaskProgress(taskType);

        // Carry out any task completion behaviour
        std_msgs::Empty emptyMessage;
        taskCompleted(emptyMessage);
    } else {
        ROS_INFO("Resident: Continue with the %s task.",  taskName.c_str());
        result = PERFORM_TASK_RESULT_ACCEPTED;
    }

    return result;
}

bool Resident::shouldRespondGoAway(int requestedTaskType) {
    bool result = false;

    if (currentTaskType == EVENT_TRIGGER_EVENT_TYPE_VERY_ILL && 
        requestedTaskType != EVENT_TRIGGER_EVENT_TYPE_VERY_ILL) {
        // We're very ill and the request is not to do with being very ill
        result = true;
    } else if (currentTaskType == EVENT_TRIGGER_EVENT_TYPE_ILL &&
        requestedTaskType != EVENT_TRIGGER_EVENT_TYPE_ILL &&
        requestedTaskType != EVENT_TRIGGER_EVENT_TYPE_VERY_ILL) {
        // We're ill and the request is unrelated to any type of illness
        result = true;
    }

    return result;
}

bool Resident::shouldOverrideCurrentTask(int requestedTaskType) {
    bool result = false;

    if (currentTaskType != EVENT_TRIGGER_EVENT_TYPE_VERY_ILL &&
        requestedTaskType == EVENT_TRIGGER_EVENT_TYPE_VERY_ILL) {
        // A VERY_ILL request should always override if our current task isn't also VERY_ILL
        result = true;
    } else if (currentTaskType != EVENT_TRIGGER_EVENT_TYPE_ILL &&
        currentTaskType != EVENT_TRIGGER_EVENT_TYPE_VERY_ILL &&
        requestedTaskType == EVENT_TRIGGER_EVENT_TYPE_ILL) {
        // An ILL request should override if the current task is not ILL or VERY_ILL
        result = true;
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
                       
    int taskType = req.taskType;
    bool taskRequiresPoi = req.taskRequiresPoi;
    geometry_msgs::Point taskPoi = req.taskPoi;

    // Sending an undefined event type is a mechanism to clear the resident's tasks
    if (taskType == EVENT_TRIGGER_RESULT_UNDEFINED) {
        clearAllTasks();
        res.result = PERFORM_TASK_RESULT_FINISHED;
        return true;
    }

    ROS_INFO("Resident: Someone is requesting to perform task %d", taskType);

    // If we're dealing with health tasks, tell other helpers to go away
    if (shouldRespondGoAway(taskType)) {
        res.result = PERFORM_TASK_RESULT_FINISHED;
        ROS_INFO("Resident: Telling them to go away.");
        return true;
    }

    // If we're dealing with a task and an illness task request comes along, switch to it
    if (shouldOverrideCurrentTask(taskType)) {
        resetTaskProgress(taskType);
        currentTaskType = taskType;
        ROS_INFO("Resident: Overriding current task.");

    }

    // No more special case needs to be considered for illness-related tasks, proceed to accept the task

    if (currentTaskType == EVENT_TRIGGER_EVENT_TYPE_UNDEFINED) {
        currentTaskType = taskType;
    }

    bool isInCorrectPlace = !taskRequiresPoi || atPointOfInterest(taskPoi, 0.5f);

    if (taskRequiresPoi && !isInCorrectPlace && !navigatingToPoiForTask) {
        navigatingToPoiForTask = true;
        goToLocation(taskPoi);
        res.result = PERFORM_TASK_RESULT_TAKE_ME_THERE;
    } else if ((taskType == currentTaskType) && isInCorrectPlace) {
        // We must be dealing with the current helper and we've reached any POI we needed to get to
        res.result = handleTask(taskType);;
    } else {
        // We are busy: either moving to a POI or dealing with another task
        res.result = PERFORM_TASK_RESULT_BUSY;
    }

    ROS_INFO("Resident: I'm responding with result %d", res.result);
        
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
    resident.goToLocation(location);
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
    ros::Rate loop_rate(10);

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

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

    return 0;

}
