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
#include "CompanionshipRobot.h"

#include "Poi.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

/**
 * This node represents an assistant robot for the purpose of
 * providing companionship to the Resident. THe robot moves to
 * the the POI, in this case the resident, to perform 
 * a predetermined action. This action represents the robot
 * providing companionship such as connecting the resident
 * to other beings via VoIP sofware (Skype).
 *
 * Author: Akshay Kalyan
 */

CompanionshipRobot::CompanionshipRobot() {
    MY_TASK = EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP;
    performingTask = false;
    currentLocationState = AT_HOME;
}

CompanionshipRobot::~CompanionshipRobot() {  
}

/**
 * Prompt Companionship robot to travel to the resident
 */
void CompanionshipRobot::goToResident(const std_msgs::Empty) {
    ROS_INFO("CompanionshipRobot: Going to %f, %f", residentPoi.getLocation().x, residentPoi.getLocation().y);
    goToLocation(residentPoi.getLocation());
    currentLocationState = GOING_TO_RESIDENT;
}

/**
 * Prompt Companionship robot to return back to its home POI
 */
void CompanionshipRobot::goToHome(const std_msgs::Empty) {
    goToLocation(homePoi.getLocation());
    currentLocationState = GOING_HOME;
}

/**
 * Publish successful completion of task notice destined
 * for the scheduler
 */
void CompanionshipRobot::eventTriggerReply() {

    // Create response message
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;
    msg.event_type = MY_TASK;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
    msg.event_weight = getEventWeight(msg.event_type);
    msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

    eventTriggerPub.publish(msg);
    ROS_INFO("CompanionshipRobot: Reply Message Sent");
}

/**
 * Perform task when a request is received. Callback executed
 * when a message is received from the Scheduler
 */
void CompanionshipRobot::eventTriggerCallback(elderly_care_simulation::EventTrigger msg)
{
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {

        if (msg.event_type == MY_TASK) {
            ROS_INFO("CompanionshipRobot: Event Recieved: [%s]", eventTypeToString(MY_TASK));
            
            performingTask = true;
            
            std_msgs::Empty emptyMessage;
            goToResident(emptyMessage);
            
        }
    }
}

/**
 * Carry out task on the resident
 */
void CompanionshipRobot::performTask() {
    
    // Generate the service call
    elderly_care_simulation::PerformTask performTaskSrv;
    performTaskSrv.request.taskType = MY_TASK;
    
    // Make the call using the client
    if (!performTaskClient.call(performTaskSrv)) {
        throw std::runtime_error("CompanionshipRobot: Service call to the initiate task with Resident failed");
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

CompanionshipRobot companionshipRobot;

// Callback functions
void callStage0domCallback(const nav_msgs::Odometry msg) {
    companionshipRobot.stage0domCallback(msg);
}
void callUpdateDesiredLocationCallback(const geometry_msgs::Point location){
    companionshipRobot.goToLocation(location);
}
void callEventTriggerCallback(elderly_care_simulation::EventTrigger msg){
    companionshipRobot.eventTriggerCallback(msg);
}
void callGoToResident(const std_msgs::Empty empty){
    ROS_INFO("CompanionshipRobot: Going to Resident");
    companionshipRobot.goToResident(empty);
}
void callGoToHome(const std_msgs::Empty empty){
    ROS_INFO("CompanionshipRobot: Going home");
    companionshipRobot.goToHome(empty);
}
void updateResidentPositionCallback(const nav_msgs::Odometry msg) {
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;    
    companionshipRobot.residentPoi = StaticPoi(x, y, 0);
}

int main(int argc, char **argv) {   
    
    // ROS initialiser calls
    ros::init(argc, argv, "CompanionshipRobot");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(25);

    companionshipRobot = CompanionshipRobot();

    // Initialise publishers
    companionshipRobot.robotNodeStagePub = nodeHandle.advertise<geometry_msgs::Twist>("robot_5/cmd_vel",1000);
    companionshipRobot.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    // Initialise subscribers
    companionshipRobot.residentStageSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000,
                                 updateResidentPositionCallback);
    companionshipRobot.stageOdoSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_5/base_pose_ground_truth",1000,
                            callStage0domCallback);
    companionshipRobot.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000,
                                callEventTriggerCallback);
    companionshipRobot.locationInstructionsSub = nodeHandle.subscribe<geometry_msgs::Point>("robot_5/location", 1000,
                                        callUpdateDesiredLocationCallback);
    companionshipRobot.pathToRobotSub = nodeHandle.subscribe<std_msgs::Empty>("robot_5/toResident", 1000, callGoToResident);
    companionshipRobot.pathToHomeSub = nodeHandle.subscribe<std_msgs::Empty>("robot_5/toHome", 1000, callGoToHome);
        
    // Create a client to make service requests to the Resident
    companionshipRobot.performTaskClient = nodeHandle.serviceClient<elderly_care_simulation::PerformTask>("perform_task");

    // Create a link to the pathfinding service
    companionshipRobot.pathFinderService = nodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");

    while (ros::ok())
    {
        // Required for dynamic pathfinding                   
        companionshipRobot.updateCurrentVelocity();

        if (companionshipRobot.atDesiredLocation()){
            if (companionshipRobot.currentLocationState == companionshipRobot.GOING_TO_RESIDENT) {
                companionshipRobot.currentLocationState = companionshipRobot.AT_RESIDENT;
            } else if (companionshipRobot.currentLocationState == companionshipRobot.GOING_HOME) {
                companionshipRobot.currentLocationState = companionshipRobot.AT_HOME;
            }       
        }

        if ((companionshipRobot.currentLocationState == companionshipRobot.AT_RESIDENT) && companionshipRobot.performingTask) {
            companionshipRobot.performTask();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
