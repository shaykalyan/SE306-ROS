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
#include "Caregiver.h"
#include "Poi.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

Caregiver::Caregiver(){
    MY_TASK = -1;
    performingTask = false;
    currentLocationState = AT_HOME;
}

Caregiver::~Caregiver(){
    
}

void Caregiver::goToResident(const std_msgs::Empty) {
    goToLocation(residentPoi.getLocation());
    ROS_INFO("CAREGIVER Going To Resident: %f %f %f", residentPoi.getLocation().x,residentPoi.getLocation().y,residentPoi.getLocation().z);    
    currentLocationState = GOING_TO_RESIDENT;
}

void Caregiver::goToHome(const std_msgs::Empty) {
    goToLocation(homePoi.getLocation());
    ROS_INFO("Caregiver Going Home");    
    currentLocationState = GOING_HOME;
}

void Caregiver::goToShower(const std_msgs::Empty){
    goToLocation(showerPoi.getLocation());
    ROS_INFO("Caregiver Going Shower: %f %f %f",showerPoi.getLocation().x,showerPoi.getLocation().y,showerPoi.getLocation().z);    
    currentLocationState = GOING_TO_SHOWER;
}

void Caregiver::eventTriggerReply() {
    // create response message
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;
    msg.event_type = MY_TASK;

     switch (msg.event_type) {
        case EVENT_TRIGGER_EVENT_TYPE_CONVERSATION:
        {
            msg.event_type = EVENT_TRIGGER_EVENT_TYPE_CONVERSATION;
            msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
            msg.event_weight = getEventWeight(msg.event_type);
            msg.result = EVENT_TRIGGER_RESULT_SUCCESS;
            eventTriggerPub.publish(msg);
            ROS_INFO("Caregiver: Reply Message Sent");
            break;
        }

         case EVENT_TRIGGER_EVENT_TYPE_SHOWER:
        {
            msg.event_type = EVENT_TRIGGER_EVENT_TYPE_SHOWER;
            msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
            msg.event_weight = getEventWeight(msg.event_type);
            msg.result = EVENT_TRIGGER_RESULT_SUCCESS;
            eventTriggerPub.publish(msg);
            ROS_INFO("Caregiver: Reply Message Sent");
            break;

        }

        case EVENT_TRIGGER_EVENT_TYPE_EXERCISE:
        {
            msg.event_type = EVENT_TRIGGER_EVENT_TYPE_EXERCISE;
            msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
            msg.event_weight = getEventWeight(msg.event_type);
            msg.result = EVENT_TRIGGER_RESULT_SUCCESS;
            eventTriggerPub.publish(msg);
            ROS_INFO("Caregiver: Reply Message Sent");
            break;

        }

        case EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT:
        {
            msg.event_type = EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT;
            msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
            msg.event_weight = getEventWeight(msg.event_type);
            msg.result = EVENT_TRIGGER_RESULT_SUCCESS;
            eventTriggerPub.publish(msg);
            ROS_INFO("Caregiver: Reply Message Sent");
            break;

        }
    }
}

void Caregiver::eventTriggerCallback(elderly_care_simulation::EventTrigger msg)
{
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {
        MY_TASK = msg.event_type;
        // TODO: NEW ROBOT CHANGE HERE
        switch (msg.event_type) {
            case EVENT_TRIGGER_EVENT_TYPE_CONVERSATION:
            {
                ROS_INFO("Caregiver: Event Recieved: [%s]", eventTypeToString(MY_TASK));
                performingTask = true;
                std_msgs::Empty emptyMessage;
                goToResident(emptyMessage);
                break;

            }    

            case EVENT_TRIGGER_EVENT_TYPE_SHOWER:
            {
                ROS_INFO("Caregiver: Event Recieved: [%s]", eventTypeToString(MY_TASK));                
                performingTask = true;                
                std_msgs::Empty emptyMessage;
                goToResident(emptyMessage);
                break;

            } 

            case EVENT_TRIGGER_EVENT_TYPE_EXERCISE:
            {
                ROS_INFO("Caregiver: Event Recieved: [%s]", eventTypeToString(MY_TASK));                
                performingTask = true;                
                std_msgs::Empty emptyMessage;
                goToResident(emptyMessage);
                break;

            }   

            case EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT:
            {
                ROS_INFO("Caregiver: Event Recieved: [%s]", eventTypeToString(MY_TASK));                
                performingTask = true;                
                std_msgs::Empty emptyMessage;
                goToResident(emptyMessage);
                break;

            }
        }
    }
}

/**
 * Perform a task on the resident by making a service call to them.
 */
void Caregiver::performTask() {
    
    // Generate the service call
    elderly_care_simulation::PerformTask performTaskSrv;
    performTaskSrv.request.taskType = MY_TASK;

    if (MY_TASK == EVENT_TRIGGER_EVENT_TYPE_SHOWER){
        performTaskSrv.request.taskRequiresPoi = true;
        performTaskSrv.request.taskPoi = showerPoi.getLocation();
    }

    // Make the call using the client
    if (!performTaskClient.call(performTaskSrv)) {
        throw std::runtime_error("Caregiver: Service call to the initiate task with Resident failed");
    }
    
    switch (performTaskSrv.response.result) {
        case PERFORM_TASK_RESULT_ACCEPTED:
        {
            // Resident has accepted the task but keep going
            ROS_INFO("Caregiver: Resident has accepted the task but says keep going");
            startSpinning(true);        
            break;
        }

        case PERFORM_TASK_RESULT_FINISHED:
        {
            // Resident accepted the task and has had enough
            ROS_INFO("Caregiver: Resident has accepted the task and has had enough");            
            performingTask = false;            
            std_msgs::Empty emptyMessage;
            goToHome(emptyMessage);
            stopSpinning();
            eventTriggerReply();
            break;
        }

        case PERFORM_TASK_RESULT_BUSY:
        {
            // Resident is busy
            ROS_INFO("Caregiver: Resident is busy");
            break;
        }

        case PERFORM_TASK_RESULT_TAKE_ME_THERE:
        {
            ROS_INFO("Caregiver: Resident is going to the shower");       
            std_msgs::Empty emptyMessage;
            goToShower(emptyMessage);
            break;
        }
    }
}

Caregiver caregiver;
/*
 * This area is for functions to call the Robot callback methods 
 */
void callStage0domCallback(const nav_msgs::Odometry msg) {
    caregiver.stage0domCallback(msg);
}

void callUpdateDesiredLocationCallback(const geometry_msgs::Point location){
    caregiver.goToLocation(location);
}

void callEventTriggerCallback(elderly_care_simulation::EventTrigger msg){
    caregiver.eventTriggerCallback(msg);
}

void callGoToResident(const std_msgs::Empty empty){
    caregiver.goToResident(empty);
}

void callGoToHome(const std_msgs::Empty empty){
    caregiver.goToHome(empty);
}

void callGoToShower(const std_msgs::Empty empty){
    caregiver.goToShower(empty);
}

/*
 * Update the resident position
 */

void updateResidentPositionCallback(const nav_msgs::Odometry msg) {
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;
    caregiver.residentPoi = StaticPoi(x, y, 0);
}

int Caregiver::caregiverDoWork(){
    ros::Rate loop_rate(10);

    while (ros::ok())
    {                   
        if (atDesiredLocation()){
             // Bad place for this, but need to get it working..
            if (currentLocationState == GOING_TO_RESIDENT) {
                currentLocationState = AT_RESIDENT;
            } else if (currentLocationState == GOING_HOME) {
                currentLocationState = AT_HOME;
            } else if (currentLocationState == GOING_TO_SHOWER){
                currentLocationState = AT_SHOWER;
            }
        }

        if ((currentLocationState == AT_RESIDENT) && performingTask) {
            performTask();
        } else if ((currentLocationState == AT_SHOWER) && performingTask){
            performTask();
        }

        updateCurrentVelocity();
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

int main(int argc, char **argv)
{   
    //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
    ros::init(argc, argv, "Caregiver");

    //NodeHandle is the main access point to communicate with ros.
    ros::NodeHandle nodeHandle;

    caregiver = Caregiver();

    caregiver.residentStageSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000,
    updateResidentPositionCallback);

    //advertise() function will tell ROS that you want to publish on a given topic_
    //to stage
    caregiver.robotNodeStagePub = nodeHandle.advertise<geometry_msgs::Twist>("robot_9/cmd_vel",1000);
    caregiver.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

    //subscribe to listen to messages coming from stage
    caregiver.stageOdoSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_9/base_pose_ground_truth",1000, callStage0domCallback);
    caregiver.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, callEventTriggerCallback);
    caregiver.locationInstructionsSub = nodeHandle.subscribe<geometry_msgs::Point>("robot_9/location", 1000, callUpdateDesiredLocationCallback);
    // caregiver.pathToRobotSub = nodeHandle.subscribe<std_msgs::Empty>("robot_9/toResident", 1000, callGoToResident);
    // caregiver.pathToHomeSub = nodeHandle.subscribe<std_msgs::Empty>("robot_9/toHome", 1000, callGoToHome);
    // caregiver.pathToShowerSub = nodeHandle.subscribe<std_msgs::Empty>("robot_9/toShower", 1000, callGoToShower);
    caregiver.pathFinderService = nodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");
        
    // Create a client to make service requests to the Resident
    caregiver.performTaskClient = nodeHandle.serviceClient<elderly_care_simulation::PerformTask>("perform_task");
    return caregiver.caregiverDoWork();
   
}
