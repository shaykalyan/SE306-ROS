#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"
#include "EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <queue>
#include <vector>
#include <utility>
#include "EventNode.h"
#include <unistd.h> // sleep

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
const double WORLD_POS_X = 10;
const double WORLD_POS_Y = 10;
double px;
double py;
double theta;

// flag to indicate scheduler's status
bool readyToSend = true;

// globals
std::priority_queue<EventNode > eventQueue;

void ResidentEvent_callback(elderly_care_simulation::EventTrigger msg) {
	ROS_INFO("Received Message from Resident");
	int priority = 2; // default
	switch(msg.event_type) {
		case EVENT_TRIGGER_EVENT_TYPE_VISITOR:
			priority = 2;
			break;
		case EVENT_TRIGGER_EVENT_TYPE_ASSISTANT:
			priority = 2;
			break;
	}
	ROS_INFO("Adding request to queue");
	eventQueue.push(EventNode(priority, msg));
}

void EventTrigger_callback(elderly_care_simulation::EventTrigger msg) {
	
	if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_RESPONSE) {
		if(msg.result == EVENT_TRIGGER_RESULT_SUCCESS){
			if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_VISITOR) {
				ROS_INFO("Response from Visitor");
			} else if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_ASSISTANT) {
				ROS_INFO("Response from Assitant");
			}
			// reset ability to send
			readyToSend = true;
		}
	}
}

int main(int argc, char **argv) {

 	//initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = M_PI/2.0;
	px = WORLD_POS_X;
	py = WORLD_POS_Y;
	
	//Initial velocity
	linear_x = 0.0;
	angular_z = 0.0;

	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Scheduler");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	// advertise to event_trigger topic
	ros::Publisher EventTrigger_pub = n.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);

	// subscribe to event_trigger topic
	ros::Subscriber EventTrigger_sub = n.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, EventTrigger_callback);
	ros::Subscriber ResidentEvent_sub = n.subscribe<elderly_care_simulation::EventTrigger>("resident_event",1000, ResidentEvent_callback);
	
	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;
	
	ROS_INFO("Testing that info messages work...");

	while (ros::ok()) {

		if (readyToSend) {

			if(eventQueue.size() > 0) {

				// block scheduler
				readyToSend = false;

				elderly_care_simulation::EventTrigger msg;
				msg = eventQueue.top().getEventTriggerMessage();
				eventQueue.pop();

				if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_VISITOR) {
					ROS_INFO("Publishing to Visitor");
				} else if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_ASSISTANT) {
					ROS_INFO("Publishing to Assitant");
				}
				EventTrigger_pub.publish(msg);
			}
			
			
		}

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
