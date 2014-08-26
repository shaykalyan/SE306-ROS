#include <ros/ros.h>
#include <string>

#include "EscortRobot.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/FindPath.h"
#include "elderly_care_simulation/PerformTask.h"
#include "EventTriggerUtility.h"
#include "nav_msgs/Odometry.h"
#include "Robot.h"
#include "StaticPoiConstants.h"
#include "PerformTaskConstants.h" 

EscortRobot::EscortRobot() {
    // No implementation
}

EscortRobot::EscortRobot(int escortEventType, geometry_msgs::Point escortBase, geometry_msgs::Point escortPoi) {
    currentLocationState = AT_BASE;
    eventType = escortEventType;
    base = escortBase;
    poi = escortPoi;
}

EscortRobot::~EscortRobot() {}

void EscortRobot::residentLocationCallback(nav_msgs::Odometry msg) {
    residentLocation = msg.pose.pose.position;
}

void EscortRobot::goToPoi() {
    currentLocationState = GOING_TO_POI;
    goToLocation(poi);
}

void EscortRobot::goToResident() {
    currentLocationState = GOING_TO_POI;
    goToLocation(residentLocation);
}

void EscortRobot::goToBase() {
    currentLocationState = GOING_TO_BASE;
    goToLocation(base);
}

void EscortRobot::eventTriggered(const elderly_care_simulation::EventTrigger msg) {
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {
        if (msg.event_type == eventType) {
            if (currentLocationState == AT_BASE ||
                currentLocationState == GOING_TO_BASE) {
                
                currentLocationState = GOING_TO_POI;
                performingTask = true;
                goToResident();
                
            }            
        }
    }
}

void EscortRobot::eventFinished() {

    stopSpinning();
    goToBase();

    // create response message
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_COOK;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
    msg.event_weight = getEventWeight(msg.event_type);
    msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

    eventTriggerPub.publish(msg);
}

/**
 * Perform a task on the resident by making a service call to them.
 */
void EscortRobot::performTask() {
    
    // Generate the service call
    elderly_care_simulation::PerformTask performTaskService;
    performTaskService.request.taskType = eventType;
    performTaskService.request.taskRequiresPoi = true;

    performTaskService.request.taskPoi = poi;

    // Make the call using the client
    if (!performTaskClient.call(performTaskService)) {
        throw std::runtime_error("Service call to the initiate task with Resident failed");
    }
    
    switch (performTaskService.response.result) {
        case PERFORM_TASK_RESULT_ACCEPTED:
        {
            startSpinning(false);
            ROS_INFO("Resident has accepted the task but says keep going");
            break;
        }
        case PERFORM_TASK_RESULT_TAKE_ME_THERE: 
        {
            currentLocationState = GOING_TO_POI;
            goToLocation(poi);
            ROS_INFO("Resident wants to be taken to the POI");
            break;
        }
        case PERFORM_TASK_RESULT_FINISHED:
        {            
            performingTask = false;
            currentLocationState = GOING_TO_BASE;
            
            goToBase();
            
            stopSpinning();
            notifySchedulerOfTaskCompletion();

            ROS_INFO("Resident has accepted the task and has had enough");

            break;
        }
        case PERFORM_TASK_RESULT_BUSY:
        {
            ROS_INFO("Resident is busy");
            break;
        }
    }
}

void EscortRobot::notifySchedulerOfTaskCompletion() {
    // Create response message
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;

    msg.event_type = eventType;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
    msg.event_weight = getEventWeight(msg.event_type);
    msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

    eventTriggerPub.publish(msg);
    ROS_INFO("Task Completion Message Sent");
}

int EscortRobot::execute() {    
    ros::Rate loopRate(10);

    while (ros::ok()) {
        if (atDesiredLocation()) {
            switch (currentLocationState) {
                case GOING_TO_BASE:
                    currentLocationState = AT_BASE;
                    break;
                case GOING_TO_POI:
                    currentLocationState = AT_POI;
                    break;
                default:
                    break;
            }

            if (performingTask && currentLocationState == AT_POI) {
                performTask();
            }
        }

        updateCurrentVelocity();

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}

EscortRobot feeder;

void eventTriggeredCallback(elderly_care_simulation::EventTrigger msg) {
    feeder.eventTriggered(msg);
}

void stageCallBack(nav_msgs::Odometry msg) {
    feeder.stage0domCallback(msg);
}

void residentStageCallback(nav_msgs::Odometry msg) {
    feeder.residentLocationCallback(msg);
}

int main(int argc, char **argv) {

    geometry_msgs::Point base;
    base.x = 12.0f;
    base.y = 1.0f;

    geometry_msgs::Point table;
    table.x = ADJACENT_TABLE_X;
    table.y = ADJACENT_TABLE_Y;

    feeder = EscortRobot(EVENT_TRIGGER_EVENT_TYPE_EAT, base, table);

    const std::string rid = "robot_10";

    ros::init(argc, argv, "Feeding_Robot");

    // Node handle
    ros::NodeHandle chefNodeHandle;
    
    // Will publish geometry_msgs::Twist messages to the cmd_vel topic
    feeder.robotNodeStagePub = chefNodeHandle.advertise<geometry_msgs::Twist>(rid + "/cmd_vel", 1000);
    feeder.eventTriggerPub = chefNodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    feeder.stageOdoSub = chefNodeHandle.subscribe<nav_msgs::Odometry>(rid + "/base_pose_ground_truth", 1000, stageCallBack);
    feeder.eventTriggerSub = chefNodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger", 1000, eventTriggeredCallback);
    feeder.residentLocationSub = chefNodeHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth", 1000, residentStageCallback);

    // Service used to find paths
    feeder.pathFinderService = chefNodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");
    
    // Service to perform tasks on the resident
    feeder.performTaskClient = chefNodeHandle.serviceClient<elderly_care_simulation::PerformTask>("perform_task");

    return feeder.execute();
}