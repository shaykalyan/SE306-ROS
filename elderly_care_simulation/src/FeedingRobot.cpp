#include <ros/ros.h>
#include <string>

#include "EscortRobot.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/FindPath.h"
#include "elderly_care_simulation/PerformTask.h"
#include "EventTriggerUtility.h"
#include "nav_msgs/Odometry.h"
#include "StaticPoiConstants.h"
#include "PerformTaskConstants.h" 
#include "Robot.h"

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
    base.x = FEEDER_BASE_X;
    base.y = FEEDER_BASE_Y;

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