#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <queue>

#include <sstream>
#include "math.h"
#include "EventTriggerUtility.h"
#include "PerformTaskConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/PerformTask.h"
#include <unistd.h>

// Tasks
const int MY_TASK = EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT;
bool performingTask = false;

// Topics
ros::Publisher robotNodeStagePub;
ros::Publisher eventTriggerPub;
ros::Subscriber eventTriggerSub;
ros::Subscriber pathToRobotSub;
ros::Subscriber pathToHomeSub;
ros::Subscriber stageOdoSub;
ros::Subscriber locationInstructionsSub;

// Services
ros::ServiceClient performTaskClient;

// Location State
enum LocationState {
	AT_HOME, AT_RESIDENT, GOING_TO_RESIDENT, GOING_HOME
};

/*************************
 * Location variables
 ************************/

double currentAngle;

// Current velocity of the Robot
geometry_msgs::Twist currentVelocity;

// Current location of the robot
geometry_msgs::Pose currentLocation;

// Locations to visit
std::queue<geometry_msgs::Point> locationQueue;

// Current state
LocationState currentLocationState = AT_HOME;

void goToResident(const std_msgs::Empty) {
    geometry_msgs::Point locationOne;
    locationOne.x = -8;
    locationOne.y = 5;
    
    geometry_msgs::Point locationTwo;
    locationTwo.x = -8;
    locationTwo.y = 3;
    
    geometry_msgs::Point locationThree;
    locationThree.x = 0;
    locationThree.y = 2;
    
    geometry_msgs::Point locationFour;
    locationFour.x = 0;
    locationFour.y = 1;
    
    locationQueue.push(locationOne);
    locationQueue.push(locationTwo);
    locationQueue.push(locationThree);
    locationQueue.push(locationFour);
    
    currentLocationState = GOING_TO_RESIDENT;
}

void goToHome(const std_msgs::Empty) {
	geometry_msgs::Point locationOne;
    locationOne.x = 0;
    locationOne.y = 2;
    
    geometry_msgs::Point locationTwo;
    locationTwo.x = -8;
    locationTwo.y = 3;
    
    geometry_msgs::Point locationThree;
    locationThree.x = -8;
    locationThree.y = 5;
    
    geometry_msgs::Point locationFour;
    locationFour.x = 7.5;
    locationFour.y = 7.5;
    
    locationQueue.push(locationOne);
    locationQueue.push(locationTwo);
    locationQueue.push(locationThree);
    locationQueue.push(locationFour);
    
    currentLocationState = GOING_HOME;

}

void stageOdometryCallback(const nav_msgs::Odometry msg)
{
    //Update Current Position
    currentLocation = msg.pose.pose;
    double x = currentLocation.orientation.x;
    double y = currentLocation.orientation.y;
    double z = currentLocation.orientation.z;
    double w = currentLocation.orientation.w;
    
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
    currentAngle = yaw;
}

void updateDesiredLocationCallback(const geometry_msgs::Point location)
{   
    // Add location to the locationQueue queue
    locationQueue.push(location);
}

bool doubleEquals(double a, double b, double difference)
{
    return std::abs(a - b) < difference;
}

double normalizeAngle(double angle)
{
    while (angle < 0) {
        angle += 2 * M_PI;
    }
    while (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}

bool turnAnticlockwise(double currentAngle, double desiredAngle)
{   
    if (currentAngle < 0) {
        currentAngle = 2 * M_PI + currentAngle;
    }
    if (desiredAngle < 0) {
        desiredAngle = 2 * M_PI + desiredAngle;
    }
    desiredAngle = normalizeAngle(desiredAngle - currentAngle);
    return desiredAngle < M_PI;
    
}

bool atDesiredLocation()
{  
    if (locationQueue.empty()) {
        return true;
    } else {
        double toleratedDifference = 0.05;
        geometry_msgs::Point desiredLocation = locationQueue.front();

        if( doubleEquals(currentLocation.position.x, desiredLocation.x, toleratedDifference) &&
            doubleEquals(currentLocation.position.y, desiredLocation.y, toleratedDifference)) {
            locationQueue.pop();
            return atDesiredLocation();
        }
    }
    return false;
      
}

void updateCurrentVelocity()
{
    if (atDesiredLocation()) {
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = 0;
        
        // Bad place for this, but need to get it working..
        if (currentLocationState == GOING_TO_RESIDENT) {
			currentLocationState = AT_RESIDENT;
		} else if (currentLocationState == GOING_HOME) {
			currentLocationState = AT_HOME;
		}
        
        return;
    }
    // Find the correct angle
    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation

    geometry_msgs::Point desiredLocation = locationQueue.front();

    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;
    
    // Thank god we're only doing 2D stuff
    double desiredAngle = atan2(directionVector.y, directionVector.x);

    if (! doubleEquals(currentAngle, desiredAngle, 0.1)) {
        // Turn towards angle
        currentVelocity.linear.x = 0;
        
        if (turnAnticlockwise(currentAngle, desiredAngle)) {
            // Turn anti clockwise
            currentVelocity.angular.z = 1;
        } else {
            // Turn clockwise
            currentVelocity.angular.z = -1;
        }
    } else {
        // Go forward
        currentVelocity.linear.x = 1;
        currentVelocity.angular.z = 0;
    }
}

void eventTriggerReply() {
	// create response message
	elderly_care_simulation::EventTrigger msg;
	msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;

    // TODO: INSERT SWITCH STATEMENT FOR DIFFERENT TASKS HERE!
	msg.event_type = EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
	msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

	eventTriggerPub.publish(msg);
	ROS_INFO("Visitor: Reply Message Sent");
}

/**
 * Send a message to Stage to start rotation of this robot.
 */
void startRotating() {
    ROS_INFO("Visitor: Started Rotating");
	currentVelocity.linear.x = 0;
	currentVelocity.angular.z = -2.0;
}

/**
 * Send a message to Stage to stop rotation of this robot.
 */
void stopRotating() {
    ROS_INFO("Visitor: Stopped Rotating");
	currentVelocity.linear.x = 0;
	currentVelocity.angular.z = 0.0;
}

void eventTriggerCallback(elderly_care_simulation::EventTrigger msg)
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
void performTask() {
	
	// Generate the service call
	elderly_care_simulation::PerformTask performTaskSrv;
	performTaskSrv.request.taskType = MY_TASK;
	
	// Make the call using the client
	if (!performTaskClient.call(performTaskSrv)) {
		throw std::runtime_error("Visitor: Service call to the initiate task with Resident failed");
	}
	
	switch (performTaskSrv.response.result) {
		case PERFORM_TASK_RESULT_ACCEPTED:
		{
			// Resident has accepted the task but keep going
			ROS_INFO("Visitor: Resident has accepted the task but says keep going");
			startRotating();
			break;
		}
		case PERFORM_TASK_RESULT_FINISHED:
		{
			// Resident accepted the task and has had enough
			ROS_INFO("Visitor: Resident has accepted the task and has had enough");
			
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
			ROS_INFO("Visitor: Resident is busy");
			break;
		}
	}
}

int main(int argc, char **argv)
{	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Visitor");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle nodeHandle;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	robotNodeStagePub = nodeHandle.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000);
	eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger", 1000, true);

	//subscribe to listen to messages coming from stage
	stageOdoSub = nodeHandle.subscribe<nav_msgs::Odometry>("robot_1/base_pose_ground_truth",1000, stageOdometryCallback);
	eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, eventTriggerCallback);
    locationInstructionsSub = nodeHandle.subscribe<geometry_msgs::Point>("robot_1/location", 1000, updateDesiredLocationCallback);
    pathToRobotSub = nodeHandle.subscribe<std_msgs::Empty>("robot_1/toResident", 1000, goToResident);
    pathToHomeSub = nodeHandle.subscribe<std_msgs::Empty>("robot_1/toHome", 1000, goToHome);
    
	
	// Create a client to make service requests to the Resident
	performTaskClient = nodeHandle.serviceClient<elderly_care_simulation::PerformTask>("perform_task");


	ros::Rate loop_rate(25);

	while (ros::ok())
	{        	        
        updateCurrentVelocity();
                
        if ((currentLocationState == AT_RESIDENT) && performingTask) {
            performTask();
        }

		robotNodeStagePub.publish(currentVelocity);
		
        ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
