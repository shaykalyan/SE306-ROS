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
#include "MedicationRobot.h"

#include "Poi.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

/**
 * Represents a medication providing robot for the Resident. Moves to
 * the Resident to perform a simple action when it is requested.
 *
 * Author: James Butler
 */

;MedicationRobot::MedicationRobot() {
    MY_TASK = EVENT_TRIGGER_EVENT_TYPE_MEDICATION;
    performingTask = false;
    currentLocationState = AT_HOME;
}

MedicationRobot::~MedicationRobot() {  
}

/**
 * Request robot to start moving towards the Resident
 */
void MedicationRobot::goToResident(const std_msgs::Empty) {
    ROS_INFO("MedicationRobot: Going to %f, %f", residentPoi.getLocation().x, residentPoi.getLocation().y);
    goToLocation(residentPoi.getLocation());
    currentLocationState = GOING_TO_RESIDENT;
}

/**
 * Request robot to move back to its home location
 */
void MedicationRobot::goToHome(const std_msgs::Empty) {
    goToLocation(homePoi.getLocation());
    currentLocationState = GOING_HOME;
}

/**
 * Publish completion report of this robot's task
 */
void MedicationRobot::eventTriggerReply() {
    // Create response message
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;
    msg.event_type = MY_TASK;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
    msg.event_weight = getEventWeight(msg.event_type);
    msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

    eventTriggerPub.publish(msg);
    ROS_INFO("MedicationRobot: Reply Message Sent");
}

/**
 * Perform task when a request is received. This callback
 * method should be called when getting a request from
 * the Scheduler. 
 */
void MedicationRobot::eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {

        if (msg.event_type == MY_TASK) {
            ROS_INFO("MedicationRobot: Event Recieved: [%s]", eventTypeToString(MY_TASK));
            
            performingTask = true;
            
            std_msgs::Empty emptyMessage;
            goToResident(emptyMessage);
            
        }
    }
}

/**
 * Perform a task on the resident by making a service call to them.
 */
void MedicationRobot::performTask() {
    
    // Generate the service call
    elderly_care_simulation::PerformTask performTaskSrv;
    performTaskSrv.request.taskType = MY_TASK;
    
    // Make the call using the client
    if (!performTaskClient.call(performTaskSrv)) {
        throw std::runtime_error("MedicationRobot: Service call to the initiate task with Resident failed");
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

MedicationRobot theMedicationRobot;

// Callback functions that wrap method invocations
void callStage0domCallback(const nav_msgs::Odometry msg) {
    theMedicationRobot.stage0domCallback(msg);
}
void callUpdateDesiredLocationCallback(const geometry_msgs::Point location){
    theMedicationRobot.goToLocation(location);
}
void callEventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    theMedicationRobot.eventTriggerCallback(msg);
}
void callGoToResident(const std_msgs::Empty empty) {
    ROS_INFO("MedicationRobot: Going to Resident");
    theMedicationRobot.goToResident(empty);
}
void callGoToHome(const std_msgs::Empty empty) {
    ROS_INFO("MedicationRobot: Going home");
    theMedicationRobot.goToHome(empty);
}
void updateResidentPositionCallback(const nav_msgs::Odometry msg) {
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;    
    theMedicationRobot.residentPoi = StaticPoi(x, y, 0);
}

int main(int argc, char **argv) {   
    
    // ROS initialiser calls
    ros::init(argc, argv, "MedicationRobot");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(25);

    theMedicationRobot = MedicationRobot();

    // Initialise publishers
    theMedicationRobot.robotNodeStagePub = nodeHandle.advertise<geometry_msgs::Twist>("robot_7/cmd_vel",1000);
    theMedicationRobot.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    // Initialise subscribers
    theMedicationRobot.residentStageSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000,
                                 updateResidentPositionCallback);
    theMedicationRobot.stageOdoSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_7/base_pose_ground_truth",1000,
                            callStage0domCallback);
    theMedicationRobot.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000,
                                callEventTriggerCallback);
    theMedicationRobot.locationInstructionsSub = nodeHandle.subscribe<geometry_msgs::Point>("robot_7/location", 1000,
                                        callUpdateDesiredLocationCallback);
    theMedicationRobot.pathToRobotSub = nodeHandle.subscribe<std_msgs::Empty>("robot_7/toResident", 1000, callGoToResident);
    theMedicationRobot.pathToHomeSub = nodeHandle.subscribe<std_msgs::Empty>("robot_7/toHome", 1000, callGoToHome);
        
    // Create a client to make service requests to the Resident
    theMedicationRobot.performTaskClient = nodeHandle.serviceClient<elderly_care_simulation::PerformTask>("perform_task");

    // Create a link to the pathfinding service
    theMedicationRobot.pathFinderService = nodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");

    while (ros::ok()) {

        // Required for dynamic pathfinding                   
        theMedicationRobot.updateCurrentVelocity();

        if (theMedicationRobot.atDesiredLocation()) {
            if (theMedicationRobot.currentLocationState == theMedicationRobot.GOING_TO_RESIDENT) {
                theMedicationRobot.currentLocationState = theMedicationRobot.AT_RESIDENT;
            } else if (theMedicationRobot.currentLocationState == theMedicationRobot.GOING_HOME) {
                theMedicationRobot.currentLocationState = theMedicationRobot.AT_HOME;
            }       
        }

        if ((theMedicationRobot.currentLocationState == theMedicationRobot.AT_RESIDENT) && theMedicationRobot.performingTask) {
            theMedicationRobot.performTask();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}
