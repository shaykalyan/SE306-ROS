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

EscortRobot doctor;

void eventTriggeredCallback(elderly_care_simulation::EventTrigger msg) {
    doctor.eventTriggered(msg);
}

void stageCallBack(nav_msgs::Odometry msg) {
    doctor.stage0domCallback(msg);
}

void residentStageCallback(nav_msgs::Odometry msg) {
    doctor.residentLocationCallback(msg);
}

int main(int argc, char **argv) {

    geometry_msgs::Point base;
    base.x = DOCTOR_BASE_X;
    base.y = DOCTOR_BASE_Y;

    geometry_msgs::Point hospital;
    hospital.x = ADJACENT_HOSPITAL_X;
    hospital.y = ADJACENT_HOSPITAL_Y;

    doctor = EscortRobot(EVENT_TRIGGER_EVENT_TYPE_VERY_ILL, base, hospital);

    const std::string rid = "robot_12";

    ros::init(argc, argv, "Doctor_Robot");

    // Node handle
    ros::NodeHandle chefNodeHandle;
    
    // Will publish geometry_msgs::Twist messages to the cmd_vel topic
    doctor.robotNodeStagePub = chefNodeHandle.advertise<geometry_msgs::Twist>(rid + "/cmd_vel", 1000);
    doctor.eventTriggerPub = chefNodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    doctor.stageOdoSub = chefNodeHandle.subscribe<nav_msgs::Odometry>(rid + "/base_pose_ground_truth", 1000, stageCallBack);
    doctor.eventTriggerSub = chefNodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger", 1000, eventTriggeredCallback);
    doctor.residentLocationSub = chefNodeHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth", 1000, residentStageCallback);

    // Service used to find paths
    doctor.pathFinderService = chefNodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");
    
    // Service to perform tasks on the resident
    doctor.performTaskClient = chefNodeHandle.serviceClient<elderly_care_simulation::PerformTask>("perform_task");

    return doctor.execute();
}