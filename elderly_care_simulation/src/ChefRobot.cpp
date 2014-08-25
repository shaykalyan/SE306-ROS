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

void ChefRobot::goToStove() {
    currentLocationState = GOING_TO_STOVE;
    goToLocation(stove.getLocation());
}

void ChefRobot::goToBase() {
    currentLocationState = GOING_TO_BASE;
    goToLocation(base.getLocation());
}

void ChefRobot::eventTriggered(const elderly_care_simulation::EventTrigger msg) {
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
                        stopSpinning();
                        goToBase();
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

ChefRobot chef;

void eventTriggeredCallback(elderly_care_simulation::EventTrigger msg) {
    chef.eventTriggered(msg);
}

void stageCallBack(nav_msgs::Odometry msg) {
    chef.stage0domCallback(msg);
}

int main(int argc, char **argv) {

    const std::string rid = "robot_4";

    ros::init(argc, argv, "Chef_Robot");

    // Node handle
    ros::NodeHandle chefNodeHandle;
    
    // Will publish geometry_msgs::Twist messages to the cmd_vel topic
    chef.robotNodeStagePub = chefNodeHandle.advertise<geometry_msgs::Twist>(rid + "/cmd_vel", 1000);

    chef.stageOdoSub = chefNodeHandle.subscribe<nav_msgs::Odometry>(rid + "/base_pose_ground_truth", 1000, eventTriggeredCallback);
    ros::Subscriber eventTriggerSub = chefNodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger", 1000, stageCallBack);

    // Service used to find paths
    chef.pathFinderService = chefNodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");

    return chef.execute();
}