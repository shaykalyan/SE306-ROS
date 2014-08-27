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
#include "NurseRobot.h"

#include "Poi.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

/**
 * This node represents a nurse robot for the purpose of
 * providing medical assistant to the Resident. The robot moves to
 * the the POI, in this case the resident, to perform 
 * a predetermined action. This action represents the robot
 * providing medical attentions when the resident is ill.
 *
 * Author: Bert Huang
 */

Nurse::Nurse() {
    MY_TASK = EVENT_TRIGGER_EVENT_TYPE_ILL;
    performingTask = false;
    currentLocationState = AT_HOME;
}

Nurse::~Nurse() {  
}

/**
 * Prompt Nurse robot to travel to the resident
 */
void Nurse::goToResident(const std_msgs::Empty) {
    ROS_INFO("Nurse: Going to %f, %f", residentPoi.getLocation().x, residentPoi.getLocation().y);
    goToLocation(residentPoi.getLocation());
    currentLocationState = GOING_TO_RESIDENT;
}

/**
 * Prompt Nurse robot to return back to its home POI
 */
void Nurse::goToHome(const std_msgs::Empty) {
    goToLocation(homePoi.getLocation());
    currentLocationState = GOING_HOME;
}

/**
 * Publish successful completion of task notice destined
 * for the scheduler
 */
void Nurse::eventTriggerReply() {

    // Create response message
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;
    msg.event_type = MY_TASK;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
    msg.event_weight = getEventWeight(msg.event_type);
    msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

    eventTriggerPub.publish(msg);
    ROS_INFO("Nurse: Reply Message Sent");
}

/**
 * Perform task when a request is received. Callback executed
 * when a message is received from the Scheduler
 */
void Nurse::eventTriggerCallback(elderly_care_simulation::EventTrigger msg)
{
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {

        if (msg.event_type == MY_TASK) {
            ROS_INFO("Nurse: Event Recieved: [%s]", eventTypeToString(MY_TASK));
            
            performingTask = true;
            
            std_msgs::Empty emptyMessage;
            goToResident(emptyMessage);
            
        }
    }
}

/**
 * Carry out task on the resident
 */
void Nurse::performTask() {
    
    // Generate the service call
    elderly_care_simulation::PerformTask performTaskSrv;
    performTaskSrv.request.taskType = MY_TASK;
    
    // Make the call using the client
    if (!performTaskClient.call(performTaskSrv)) {
        throw std::runtime_error("Nurse: Service call to the initiate task with Resident failed");
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

Nurse nurseRobot;

// Callback functions
void callStage0domCallback(const nav_msgs::Odometry msg) {
    nurseRobot.stage0domCallback(msg);
}
void callUpdateDesiredLocationCallback(const geometry_msgs::Point location){
    nurseRobot.goToLocation(location);
}
void callEventTriggerCallback(elderly_care_simulation::EventTrigger msg){
    nurseRobot.eventTriggerCallback(msg);
}
void callGoToResident(const std_msgs::Empty empty){
    ROS_INFO("Nurse: Going to Resident");
    nurseRobot.goToResident(empty);
}
void callGoToHome(const std_msgs::Empty empty){
    ROS_INFO("Nurse: Going home");
    nurseRobot.goToHome(empty);
}
void updateResidentPositionCallback(const nav_msgs::Odometry msg) {
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;    
    nurseRobot.residentPoi = StaticPoi(x, y, 0);
}

int main(int argc, char **argv) {   
    
    // ROS initialiser calls
    ros::init(argc, argv, "Nurse");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(25);

    nurseRobot = Nurse();

    // Initialise publishers
    nurseRobot.robotNodeStagePub = nodeHandle.advertise<geometry_msgs::Twist>("robot_8/cmd_vel",1000);
    nurseRobot.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    // Initialise subscribers
    nurseRobot.residentStageSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000,
                                 updateResidentPositionCallback);
    nurseRobot.stageOdoSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_8/base_pose_ground_truth",1000,
                            callStage0domCallback);
    nurseRobot.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000,
                                callEventTriggerCallback);
    nurseRobot.locationInstructionsSub = nodeHandle.subscribe<geometry_msgs::Point>("robot_8/location", 1000,
                                        callUpdateDesiredLocationCallback);
    nurseRobot.pathToRobotSub = nodeHandle.subscribe<std_msgs::Empty>("robot_8/toResident", 1000, callGoToResident);
    nurseRobot.pathToHomeSub = nodeHandle.subscribe<std_msgs::Empty>("robot_8/toHome", 1000, callGoToHome);
        
    // Create a client to make service requests to the Resident
    nurseRobot.performTaskClient = nodeHandle.serviceClient<elderly_care_simulation::PerformTask>("perform_task");

    // Create a link to the pathfinding service
    nurseRobot.pathFinderService = nodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");

    while (ros::ok())
    {
        // Required for dynamic pathfinding                   
        nurseRobot.updateCurrentVelocity();

        if (nurseRobot.atDesiredLocation()){
            if (nurseRobot.currentLocationState == nurseRobot.GOING_TO_RESIDENT) {
                nurseRobot.currentLocationState = nurseRobot.AT_RESIDENT;
            } else if (nurseRobot.currentLocationState == nurseRobot.GOING_HOME) {
                nurseRobot.currentLocationState = nurseRobot.AT_HOME;
            }       
        }

        if ((nurseRobot.currentLocationState == nurseRobot.AT_RESIDENT) && nurseRobot.performingTask) {
            nurseRobot.performTask();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}