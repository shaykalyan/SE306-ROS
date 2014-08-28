#include "ros/ros.h"
#include <unistd.h>
#include <sstream>

#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

#include <tf/tf.h>
#include <queue>

#include "EventTriggerUtility.h"
#include "PerformTaskConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/PerformTask.h"
#include "elderly_care_simulation/FindPath.h"

#include "Robot.h"
#include "FriendRobot.h"

#include "Poi.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

/**
 * Represents a friend (visitor) of the Resident. Moves to
 * the Resident to perform a simple action when it is requested.
 *
 * Author: Matthew Chiam
 */

;FriendRobot::FriendRobot() {
    MY_TASK = EVENT_TRIGGER_EVENT_TYPE_FRIEND;
    performingTask = false;
    currentLocationState = AT_HOME;
}

FriendRobot::~FriendRobot() {  
}

/**
 * Request robot to start moving towards the Resident
 */
void FriendRobot::goToResident(const std_msgs::Empty) {
    ROS_INFO("Friend: Going to %f, %f", residentPoi.getLocation().x, residentPoi.getLocation().y);
    goToLocation(residentPoi.getLocation(), true);
    currentLocationState = GOING_TO_RESIDENT;
}

/**
 * Request robot to move back to its home location
 */
void FriendRobot::goToHome(const std_msgs::Empty) {
    goToLocation(homePoi.getLocation());
    currentLocationState = GOING_HOME;
}

/**
 * Publish completion report of this robot's task
 */
void FriendRobot::eventTriggerReply() {

    // Create response message
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;
    msg.event_type = MY_TASK;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
    msg.event_weight = getEventWeight(msg.event_type);
    msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

    eventTriggerPub.publish(msg);
    ROS_INFO("FriendRobot: Reply Message Sent");
}

/**
 * Perform task when a request is received. This callback
 * method should be called when getting a request from
 * the Scheduler. 
 */
void FriendRobot::eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {

        if (msg.event_type == MY_TASK) {
            ROS_INFO("Friend: Event Recieved: [%s]", eventTypeToString(MY_TASK));
            
            performingTask = true;
            
            std_msgs::Empty emptyMessage;
            goToResident(emptyMessage);
            
        }
    }
}

/**
 * Perform a task on the resident by making a service call to them.
 */
void FriendRobot::performTask() {
    
    // Generate the service call
    elderly_care_simulation::PerformTask performTaskSrv;
    performTaskSrv.request.taskType = MY_TASK;
    
    // Make the call using the client
    if (!performTaskClient.call(performTaskSrv)) {
        throw std::runtime_error("Friend: Service call to the initiate task with Resident failed");
    }
    
    switch (performTaskSrv.response.result) {
        case PERFORM_TASK_RESULT_ACCEPTED:
        {
            // Resident has accepted the task but isn't finished yet
            startSpinning(false);
            break;
        }
        case PERFORM_TASK_RESULT_FINISHED:
        {
            // Resident accepted the task and has had enough            
            performingTask = false;
            
            std_msgs::Empty emptyMessage;
            goToHome(emptyMessage);
            
            stopSpinning();
            eventTriggerReply();
            break;
        }
        case PERFORM_TASK_RESULT_BUSY:
        {
            // Resident is busy, do nothing
            break;
        }
    }
}

FriendRobot theFriend;

// Callback functions that wrap method invocations
void callStage0domCallback(const nav_msgs::Odometry msg) {
    theFriend.stage0domCallback(msg);
}
void callUpdateDesiredLocationCallback(const geometry_msgs::Point location){
    theFriend.goToLocation(location);
}
void callEventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    theFriend.eventTriggerCallback(msg);
}
void callGoToResident(const std_msgs::Empty empty) {
    ROS_INFO("Friend: Going to Resident");
    theFriend.goToResident(empty);
}
void callGoToHome(const std_msgs::Empty empty) {
    ROS_INFO("Friend: Going home");
    theFriend.goToHome(empty);
}
void updateResidentPositionCallback(const nav_msgs::Odometry msg) {
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;    
    theFriend.residentPoi = StaticPoi(x, y, 0);
}

int main(int argc, char **argv) {   
    
    // ROS initialiser calls
    ros::init(argc, argv, "Friend");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(25);

    theFriend = FriendRobot();

    // Initialise publishers
    theFriend.robotNodeStagePub = nodeHandle.advertise<geometry_msgs::Twist>("robot_4/cmd_vel",1000);
    theFriend.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    // Initialise subscribers
    theFriend.residentStageSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000,
                                 updateResidentPositionCallback);
    theFriend.stageOdoSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_4/base_pose_ground_truth",1000,
                            callStage0domCallback);
    theFriend.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000,
                                callEventTriggerCallback);
    theFriend.locationInstructionsSub = nodeHandle.subscribe<geometry_msgs::Point>("robot_4/location", 1000,
                                        callUpdateDesiredLocationCallback);
    theFriend.pathToRobotSub = nodeHandle.subscribe<std_msgs::Empty>("robot_4/toResident", 1000, callGoToResident);
    theFriend.pathToHomeSub = nodeHandle.subscribe<std_msgs::Empty>("robot_4/toHome", 1000, callGoToHome);
        
    // Create a client to make service requests to the Resident
    theFriend.performTaskClient = nodeHandle.serviceClient<elderly_care_simulation::PerformTask>("perform_task");

    // Create a link to the pathfinding service
    theFriend.pathFinderService = nodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");

    while (ros::ok()) {

        // Required for dynamic pathfinding                   
        theFriend.updateCurrentVelocity();

        if (theFriend.atDesiredLocation()) {
            if (theFriend.currentLocationState == theFriend.GOING_TO_RESIDENT) {
                theFriend.currentLocationState = theFriend.AT_RESIDENT;
            } else if (theFriend.currentLocationState == theFriend.GOING_HOME) {
                theFriend.currentLocationState = theFriend.AT_HOME;
            }       
        }

        if ((theFriend.currentLocationState == theFriend.AT_RESIDENT) && theFriend.performingTask) {
            theFriend.performTask();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
