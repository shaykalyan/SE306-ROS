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
#include "EntertainmentRobot.h"

#include "Poi.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

/**
 * This node represents an assistant robot for the purpose of
 * providing entertainment to the Resident. The robot moves to
 * the the POI, in this case the resident, to perform 
 * a predetermined action. This action represents the robot
 * providing entertainment.
 *
 * Author: Hugo Bateman
 */

EntertainmentRobot::EntertainmentRobot() {
    MY_TASK = EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT;
    performingTask = false;
    currentLocationState = AT_HOME;
}

EntertainmentRobot::~EntertainmentRobot() {  
}

/**
 * Prompt EntertainmentRobot robot to travel to the resident
 */
void EntertainmentRobot::goToResident(const std_msgs::Empty) {
    ROS_INFO("Entertainment: Going to %f, %f", residentPoi.getLocation().x, residentPoi.getLocation().y);
    goToLocation(residentPoi.getLocation(), true);
    currentLocationState = GOING_TO_RESIDENT;
}

/**
 * Prompt EntertainmentRobot robot to return back to its home POI
 */
void EntertainmentRobot::goToHome(const std_msgs::Empty) {
    goToLocation(homePoi.getLocation());
    currentLocationState = GOING_HOME;
}

/**
 * Publish successful completion of task notice destined
 * for the scheduler
 */
void EntertainmentRobot::eventTriggerReply() {

    // Create response message
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;
    msg.event_type = MY_TASK;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
    msg.event_weight = getEventWeight(msg.event_type);
    msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

    eventTriggerPub.publish(msg);
    ROS_INFO("Entertainment: Reply Message Sent");
}

/**
 * Perform task when a request is received. Callback executed
 * when a message is received from the Scheduler
 */
void EntertainmentRobot::eventTriggerCallback(elderly_care_simulation::EventTrigger msg)
{
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {

        if (msg.event_type == MY_TASK) {
            ROS_INFO("Entertainment: Event Recieved: [%s]", eventTypeToString(MY_TASK));
            
            performingTask = true;
            
            std_msgs::Empty emptyMessage;
            goToResident(emptyMessage);
            
        }
    }
}

/**
 * Carry out task on the resident
 */
void EntertainmentRobot::performTask() {
    
    // Generate the service call
    elderly_care_simulation::PerformTask performTaskSrv;
    performTaskSrv.request.taskType = MY_TASK;
    
    // Make the call using the client
    if (!performTaskClient.call(performTaskSrv)) {
        throw std::runtime_error("Entertainment: Service call to the initiate task with Resident failed");
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

EntertainmentRobot entertainmentRobot;

// Callback functions
void callStage0domCallback(const nav_msgs::Odometry msg) {
    entertainmentRobot.stage0domCallback(msg);
}
void callUpdateDesiredLocationCallback(const geometry_msgs::Point location){
    entertainmentRobot.goToLocation(location);
}
void callEventTriggerCallback(elderly_care_simulation::EventTrigger msg){
    entertainmentRobot.eventTriggerCallback(msg);
}
void callGoToResident(const std_msgs::Empty empty){
    ROS_INFO("Entertainment: Going to Resident");
    entertainmentRobot.goToResident(empty);
}
void callGoToHome(const std_msgs::Empty empty){
    ROS_INFO("Entertainment: Going home");
    entertainmentRobot.goToHome(empty);
}
void updateResidentPositionCallback(const nav_msgs::Odometry msg) {
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;    
    entertainmentRobot.residentPoi = StaticPoi(x, y, 0);
}

int main(int argc, char **argv) {   
    
    // ROS initialiser calls
    ros::init(argc, argv, "Entertainment");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(25);

    entertainmentRobot = EntertainmentRobot();

    // Initialise publishers
    entertainmentRobot.robotNodeStagePub = nodeHandle.advertise<geometry_msgs::Twist>("robot_8/cmd_vel",1000);
    entertainmentRobot.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    // Initialise subscribers
    entertainmentRobot.residentStageSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000,
                                 updateResidentPositionCallback);
    entertainmentRobot.stageOdoSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_8/base_pose_ground_truth",1000,
                            callStage0domCallback);
    entertainmentRobot.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000,
                                callEventTriggerCallback);
    entertainmentRobot.locationInstructionsSub = nodeHandle.subscribe<geometry_msgs::Point>("robot_8/location", 1000,
                                        callUpdateDesiredLocationCallback);
    entertainmentRobot.pathToRobotSub = nodeHandle.subscribe<std_msgs::Empty>("robot_8/toResident", 1000, callGoToResident);
    entertainmentRobot.pathToHomeSub = nodeHandle.subscribe<std_msgs::Empty>("robot_8/toHome", 1000, callGoToHome);
        
    // Create a client to make service requests to the Resident
    entertainmentRobot.performTaskClient = nodeHandle.serviceClient<elderly_care_simulation::PerformTask>("perform_task");

    // Create a link to the pathfinding service
    entertainmentRobot.pathFinderService = nodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");

    while (ros::ok())
    {
        // Required for dynamic pathfinding                   
        entertainmentRobot.updateCurrentVelocity();

        if (entertainmentRobot.atDesiredLocation()){
            if (entertainmentRobot.currentLocationState == entertainmentRobot.GOING_TO_RESIDENT) {
                entertainmentRobot.currentLocationState = entertainmentRobot.AT_RESIDENT;
            } else if (entertainmentRobot.currentLocationState == entertainmentRobot.GOING_HOME) {
                entertainmentRobot.currentLocationState = entertainmentRobot.AT_HOME;
            }       
        }

        if ((entertainmentRobot.currentLocationState == entertainmentRobot.AT_RESIDENT) && entertainmentRobot.performingTask) {
            entertainmentRobot.performTask();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}