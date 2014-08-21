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

#include "Robot.h"
#include "Assistant.h"

Assistant::Assistant(){
    MY_TASK = EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT;
    performingTask = false;
    currentLocationState = AT_HOME;
}

Assistant::~Assistant(){
    
}

void Assistant::goToResident(const std_msgs::Empty) {
    geometry_msgs::Point locationOne;
    locationOne.x = -4;
    locationOne.y = 2;
    
    geometry_msgs::Point locationTwo;
    locationTwo.x = 0;
    locationTwo.y = 2;
    
    geometry_msgs::Point locationThree;
    locationThree.x = 0;
    locationThree.y = 1;
    
    locationQueue.push(locationOne);
    locationQueue.push(locationTwo);
    locationQueue.push(locationThree);
    
    currentLocationState = GOING_TO_RESIDENT;
}

void Assistant::goToHome(const std_msgs::Empty) {
    geometry_msgs::Point locationOne;
    locationOne.x = 0;
    locationOne.y = 2;
    
    geometry_msgs::Point locationTwo;
    locationTwo.x = -4;
    locationTwo.y = 2;
    
    geometry_msgs::Point locationThree;
    locationThree.x = -7.5;
    locationThree.y = -7.5;
    
    locationQueue.push(locationOne);
    locationQueue.push(locationTwo);
    locationQueue.push(locationThree);
    
    currentLocationState = GOING_HOME;

}

void Assistant::eventTriggerReply() {
    // create response message
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;

    // TODO: INSERT SWITCH STATEMENT FOR DIFFERENT TASKS HERE!
    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
    msg.event_weight = getEventWeight(msg.event_type);
    msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

    eventTriggerPub.publish(msg);
    ROS_INFO("Assistant: Reply Message Sent");
}

/**
 * Send a message to Stage to start rotation of this robot.
 */
void Assistant::startRotating() {
    ROS_INFO("Assistant: Started Rotating");
    currentVelocity.linear.x = 0;
    currentVelocity.angular.z = 2.0;
}

/**
 * Send a message to Stage to stop rotation of this robot.
 */
void Assistant::stopRotating() {
    ROS_INFO("Assistant: Stopped Rotating");
    currentVelocity.linear.x = 0;
    currentVelocity.angular.z = 0.0;
}

void Assistant::eventTriggerCallback(elderly_care_simulation::EventTrigger msg)
{
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {

        // TODO: NEW ROBOT CHANGE HERE
        if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT) {
            ROS_INFO("Assistant: Event Recieved: [ENTERTAINMENT]");
            
            performingTask = true;
            
            std_msgs::Empty emptyMessage;
            goToResident(emptyMessage);
            
        }
    }
}

/**
 * Perform a task on the resident by making a service call to them.
 */
void Assistant::performTask() {
    
    // Generate the service call
    elderly_care_simulation::PerformTask performTaskSrv;
    performTaskSrv.request.taskType = MY_TASK;
    
    // Make the call using the client
    if (!performTaskClient.call(performTaskSrv)) {
        throw std::runtime_error("Assistant: Service call to the initiate task with Resident failed");
    }
    
    switch (performTaskSrv.response.result) {
        case PERFORM_TASK_RESULT_ACCEPTED:
        {
            // Resident has accepted the task but keep going
            ROS_INFO("Assistant: Resident has accepted the task but says keep going");
            startRotating();
            break;
        }
        case PERFORM_TASK_RESULT_FINISHED:
        {
            // Resident accepted the task and has had enough
            ROS_INFO("Assistant: Resident has accepted the task and has had enough");
            
            performingTask = false;
            
            std_msgs::Empty emptyMessage;
            goToHome(emptyMessage);
            
            stopRotating();
            eventTriggerReply();
            break;
        }
        case PERFORM_TASK_RESULT_BUSY:
        {
            // Resident is busy
            ROS_INFO("Assistant: Resident is busy");
            break;
        }
    }
}

Assistant assistant;

void callStage0domCallback(const nav_msgs::Odometry msg) {
    assistant.stage0domCallback(msg);
}

void callUpdateDesiredLocationCallback(const geometry_msgs::Point location){
    assistant.updateDesiredLocationCallback(location);
}

void callEventTriggerCallback(elderly_care_simulation::EventTrigger msg){
    assistant.eventTriggerCallback(msg);
}

void callGoToResident(const std_msgs::Empty empty){
    assistant.goToResident(empty);
}

void callGoToHome(const std_msgs::Empty empty){
    assistant.goToHome(empty);
}

int main(int argc, char **argv)
{   
    //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
    ros::init(argc, argv, "Assistant");

    //NodeHandle is the main access point to communicate with ros.
    ros::NodeHandle nodeHandle;

    assistant = Assistant();

    //advertise() function will tell ROS that you want to publish on a given topic_
    //to stage
    assistant.robotNodeStagePub = nodeHandle.advertise<geometry_msgs::Twist>("robot_2/cmd_vel",1000);
    assistant.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    //subscribe to listen to messages coming from stage
    assistant.stageOdoSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_2/base_pose_ground_truth",1000, callStage0domCallback);
    assistant.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, callEventTriggerCallback);
    assistant.locationInstructionsSub = nodeHandle.subscribe<geometry_msgs::Point>("robot_2/location", 1000, callUpdateDesiredLocationCallback);
    assistant.pathToRobotSub = nodeHandle.subscribe<std_msgs::Empty>("robot_2/toResident", 1000, callGoToResident);
    assistant.pathToHomeSub = nodeHandle.subscribe<std_msgs::Empty>("robot_2/toHome", 1000, callGoToHome);
        
    // Create a client to make service requests to the Resident
    assistant.performTaskClient = nodeHandle.serviceClient<elderly_care_simulation::PerformTask>("perform_task");

    ros::Rate loop_rate(25);

    while (ros::ok())
    {                   
        assistant.updateCurrentVelocity();

        if (assistant.atDesiredLocation()){
             // Bad place for this, but need to get it working..
            if (assistant.currentLocationState == assistant.GOING_TO_RESIDENT) {
                assistant.currentLocationState = assistant.AT_RESIDENT;
            } else if (assistant.currentLocationState == assistant.GOING_HOME) {
                assistant.currentLocationState = assistant.AT_HOME;
            }       
        }

        if ((assistant.currentLocationState == assistant.AT_RESIDENT) && assistant.performingTask) {
            assistant.performTask();
        }

        assistant.robotNodeStagePub.publish(assistant.currentVelocity);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
