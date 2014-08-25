#include <ros/ros.h>
#include <string>

#include "ChefRobot.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/FindPath.h"
#include "EventTriggerUtility.h"
#include "nav_msgs/Odometry.h"
#include "Robot.h"


ChefRobot::ChefRobot() {
    currentLocationState = AT_BASE;
}

ChefRobot::~ChefRobot() {}

/**
 * Tell this robot to go to the stove.
 */
void ChefRobot::goToStove() {
    currentLocationState = GOING_TO_STOVE;
    goToLocation(stove.getLocation());
    ROS_INFO("GOING TO STOVE");
}

/**
 * Tell the robot to go to the base.
 */
void ChefRobot::goToBase() {
    currentLocationState = GOING_TO_BASE;
    goToLocation(base.getLocation());
    ROS_INFO("GOING TO BASE");
}

/**
 * Method to be called to start this robot cooking.
 */
void ChefRobot::eventTriggered(const elderly_care_simulation::EventTrigger msg) {
    ROS_INFO("Recieved event");
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {
        if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_COOK) {
            if (currentLocationState == AT_BASE ||
                currentLocationState == GOING_TO_BASE) {
                
                currentLocationState = GOING_TO_STOVE;
                goToStove();
            }            
        }
    }
}

/**
 * Method to be called when this robot has finished cooking.
 */
void ChefRobot::eventFinished() {

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
 * Start this robot executing.
 */
int ChefRobot::execute() {    
    ros::Rate loopRate(10);

    uint count = 0;
    const uint cookDuration = 100;

    while (ros::ok()) {
        if (atDesiredLocation()) {
            switch (currentLocationState) {
                case GOING_TO_BASE:
                    currentLocationState = AT_BASE;
                    break;
                case GOING_TO_STOVE:
                    currentLocationState = AT_STOVE;
                    break;
                case AT_STOVE:
                    if (count < cookDuration) {
                        startSpinning(true);
                        ++count;
                    } else {
                        eventFinished();
                    }
                    break;
                case AT_BASE:
                    break;
            }
        }

        updateCurrentVelocity();

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}

// The chef
ChefRobot chef;

/**
 * Helper function to pass callback function to Chef
 */
void eventTriggeredCallback(elderly_care_simulation::EventTrigger msg) {
    ROS_INFO("Recieved event outside robot");
    chef.eventTriggered(msg);
}
/**
 * Helper function to pass callback function to Chef
 */
void stageCallBack(nav_msgs::Odometry msg) {
    chef.stage0domCallback(msg);
}

/**
 * Main method. Initializes and executes a ChefRobot.
 */
int main(int argc, char **argv) {

    const std::string rid = "robot_3";

    ros::init(argc, argv, "Chef_Robot");

    // Node handle
    ros::NodeHandle chefNodeHandle;
    
    // Will publish geometry_msgs::Twist messages to the cmd_vel topic
    chef.robotNodeStagePub = chefNodeHandle.advertise<geometry_msgs::Twist>(rid + "/cmd_vel", 1000);
    chef.eventTriggerPub = chefNodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    chef.stageOdoSub = chefNodeHandle.subscribe<nav_msgs::Odometry>(rid + "/base_pose_ground_truth", 1000, stageCallBack);
    chef.eventTriggerSub = chefNodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger", 1000, eventTriggeredCallback);

    // Service used to find paths
    chef.pathFinderService = chefNodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");

    return chef.execute();
}