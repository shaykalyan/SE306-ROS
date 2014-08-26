#include <ros/ros.h>
#include <string>

#include "FeedingRobot.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/FindPath.h"
#include "EventTriggerUtility.h"
#include "nav_msgs/Odometry.h"
#include "Robot.h"
#include "StaticPoiConstants.h"

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

void EscortRobot::residentStageCallback(nav_msgs::Odometry msg) {
    residentLocation = msg.pose.pose;
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
                
                currentLocationState = GOING_TO_TABLE;
                goToPoi();
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

int EscortRobot::execute() {    
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

    feeder = EscortRobot(EVENT_TRIGGER_EVENT_TYPE_COOK, base, table);

    const std::string rid = "robot_9";

    ros::init(argc, argv, "Feeding_Robot");

    // Node handle
    ros::NodeHandle chefNodeHandle;
    
    // Will publish geometry_msgs::Twist messages to the cmd_vel topic
    feeder.robotNodeStagePub = chefNodeHandle.advertise<geometry_msgs::Twist>(rid + "/cmd_vel", 1000);
    feeder.eventTriggerPub = chefNodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    feeder.stageOdoSub = chefNodeHandle.subscribe<nav_msgs::Odometry>(rid + "/base_pose_ground_truth", 1000, stageCallBack);
    feeder.eventTriggerSub = chefNodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger", 1000, eventTriggeredCallback);
    feeder.residentLocationSub = chefNodeHanlde.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth", 1000, residentStageCallback);

    // Service used to find paths
    feeder.pathFinderService = chefNodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");

    return feeder.execute();
}