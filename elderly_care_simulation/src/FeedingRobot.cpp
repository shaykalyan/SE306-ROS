#include <ros/ros.h>
#include <string>

#include "FeedingRobot.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/FindPath.h"
#include "EventTriggerUtility.h"
#include "nav_msgs/Odometry.h"
#include "Robot.h"


FeedingRobot::FeedingRobot() {
    currentLocationState = AT_BASE;
}

FeedingRobot::~FeedingRobot() {}

void FeedingRobot::goToTable() {
    currentLocationState = GOING_TO_TABLE;
    goToLocation(table.getLocation());
}

void FeedingRobot::goToBase() {
    currentLocationState = GOING_TO_BASE;
    goToLocation(base.getLocation());
}

void FeedingRobot::eventTriggered(const elderly_care_simulation::EventTrigger msg) {
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {
        if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_COOK) {
            if (currentLocationState == AT_BASE ||
                currentLocationState == GOING_TO_BASE) {
                
                currentLocationState = GOING_TO_TABLE;
                goToTable();
            }            
        }
    }
}

void FeedingRobot::eventFinished() {

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

int FeedingRobot::execute() {    
    ros::Rate loopRate(10);

    uint count = 0;
    const uint cookDuration = 100;

    while (ros::ok()) {
        if (atDesiredLocation()) {
            switch (currentLocationState) {
                case GOING_TO_BASE:
                    currentLocationState = AT_BASE;
                    break;
                case GOING_TO_TABLE:
                    currentLocationState = AT_TABLE;
                    break;
                case AT_TABLE:
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

FeedingRobot feed;

void eventTriggeredCallback(elderly_care_simulation::EventTrigger msg) {
    feed.eventTriggered(msg);
}

void stageCallBack(nav_msgs::Odometry msg) {
    feed.stage0domCallback(msg);
}

int main(int argc, char **argv) {

    const std::string rid = "robot_3";

    ros::init(argc, argv, "Feeding_Robot");

    // Node handle
    ros::NodeHandle chefNodeHandle;
    
    // Will publish geometry_msgs::Twist messages to the cmd_vel topic
    feed.robotNodeStagePub = chefNodeHandle.advertise<geometry_msgs::Twist>(rid + "/cmd_vel", 1000);
    feed.eventTriggerPub = chefNodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    feed.stageOdoSub = chefNodeHandle.subscribe<nav_msgs::Odometry>(rid + "/base_pose_ground_truth", 1000, stageCallBack);
    feed.eventTriggerSub = chefNodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger", 1000, eventTriggeredCallback);

    // Service used to find paths
    feed.pathFinderService = chefNodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");

    return feed.execute();
}