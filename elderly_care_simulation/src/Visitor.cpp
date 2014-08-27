#include "ros/ros.h"
#include <sstream>
#include <unistd.h>

#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <queue>

#include "EventTriggerUtility.h"
#include "PerformTaskConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/PerformTask.h"
#include "elderly_care_simulation/FindPath.h"

#include "Visitor.h"
#include "Robot.h"

Visitor::Visitor(){
    MY_TASK = EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT;
    performingTask = false;
    currentLocationState = AT_HOME;
}

Visitor::~Visitor(){
    
}

void Visitor::goToResident(const std_msgs::Empty) {
    geometry_msgs::Point location;
    location.x = 0;
    location.y = 1.5;
    
    goToLocation(location);
    
    currentLocationState = GOING_TO_RESIDENT;
}

void Visitor::goToHome(const std_msgs::Empty) {
	geometry_msgs::Point location;
    location.x = 7.5;
    location.y = 7.5;
    
    goToLocation(location);
    
    currentLocationState = GOING_HOME;

}

void Visitor::eventTriggerReply() {
	// create response message
	elderly_care_simulation::EventTrigger msg;
	msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;

    // TODO: INSERT SWITCH STATEMENT FOR DIFFERENT TASKS HERE!
	msg.event_type = EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
    msg.event_weight = getEventWeight(msg.event_type);
	msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

	eventTriggerPub.publish(msg);
	ROS_INFO("Visitor: Reply Message Sent");
}

void Visitor::eventTriggerCallback(elderly_care_simulation::EventTrigger msg)
{
	if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {

        // TODO: INSERT SWITCH STATEMENT FOR DIFFERENT TASKS HERE!
		if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT) {
			ROS_INFO("Visitor: Event Recieved: [MORAL_SUPPORT]");
			
			performingTask = true;
			
			std_msgs::Empty emptyMessage;
			goToResident(emptyMessage);
			
		}
	}
}

/**
 * Perform a task on the resident by making a service call to them.
 */
void Visitor::performTask() {
	
	// Generate the service call
	elderly_care_simulation::PerformTask performTaskSrv;
	performTaskSrv.request.taskType = MY_TASK;
    //performTaskSrv.request.taskRequiresPoi = true;

    //geometry_msgs::Point taskPoi;
    //taskPoi.x = 0;
    //taskPoi.y = -4;
    //taskPoi.z = 0;

    //performTaskSrv.request.taskPoi = taskPoi;
	
	// Make the call using the client
	if (!performTaskClient.call(performTaskSrv)) {
		throw std::runtime_error("Visitor: Service call to the initiate task with Resident failed");
	}
	
	switch (performTaskSrv.response.result) {
		case PERFORM_TASK_RESULT_ACCEPTED:
		{
			// Resident has accepted the task but keep going
			ROS_INFO("Visitor: Resident has accepted the task but says keep going");
			startSpinning(false);
			break;
		}
        case PERFORM_TASK_RESULT_TAKE_ME_THERE: 
        {
            ROS_INFO("Visitor: Resident wants to be taken to the POI");
            break;
        }
		case PERFORM_TASK_RESULT_FINISHED:
		{
			// Resident accepted the task and has had enough
			ROS_INFO("Visitor: Resident has accepted the task and has had enough");
			
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
			ROS_INFO("Visitor: Resident is busy");
			break;
		}
	}
}

Visitor visitor;

void callStage0domCallback(const nav_msgs::Odometry msg) {
    visitor.stage0domCallback(msg);
}

void callUpdateDesiredLocationCallback(const geometry_msgs::Point location){
    visitor.goToLocation(location);
}

void callEventTriggerCallback(elderly_care_simulation::EventTrigger msg){
    visitor.eventTriggerCallback(msg);
}

void callGoToResident(const std_msgs::Empty empty){
    visitor.goToResident(empty);
}

void callGoToHome(const std_msgs::Empty empty){
    visitor.goToHome(empty);
}

int main(int argc, char **argv)
{	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Visitor");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle nodeHandle;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	visitor.robotNodeStagePub = nodeHandle.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000);
	visitor.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

	//subscribe to listen to messages coming from stage
	visitor.stageOdoSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_1/base_pose_ground_truth",1000, callStage0domCallback);
	visitor.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, callEventTriggerCallback);
    visitor.locationInstructionsSub = nodeHandle.subscribe<geometry_msgs::Point>("robot_1/location", 1000, callUpdateDesiredLocationCallback);
    visitor.pathToRobotSub = nodeHandle.subscribe<std_msgs::Empty>("robot_1/toResident", 1000, callGoToResident);
    visitor.pathToHomeSub = nodeHandle.subscribe<std_msgs::Empty>("robot_1/toHome", 1000, callGoToHome);
    
    visitor.pathFinderService = nodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");
	
	// Create a client to make service requests to the Resident
	visitor.performTaskClient = nodeHandle.serviceClient<elderly_care_simulation::PerformTask>("perform_task");


	ros::Rate loop_rate(10);

	while (ros::ok())
	{        	        
        
        if (visitor.atDesiredLocation()){
             // Bad place for this, but need to get it working..
            if (visitor.currentLocationState == visitor.GOING_TO_RESIDENT) {
                visitor.currentLocationState = visitor.AT_RESIDENT;
            } else if (visitor.currentLocationState == visitor.GOING_HOME) {
                visitor.currentLocationState = visitor.AT_HOME;
            }       
        }
                
        if ((visitor.currentLocationState == visitor.AT_RESIDENT) && visitor.performingTask) {
            visitor.performTask();
        }

		visitor.updateCurrentVelocity();
		
        ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
