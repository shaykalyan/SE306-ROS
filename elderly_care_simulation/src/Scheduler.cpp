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

// flag to indicate scheduler's status
bool readyToSend = true;

// globals
std::priority_queue<EventNode > eventQueue;

void residentEventCallback(elderly_care_simulation::EventTrigger msg) {
	ROS_INFO("Scheduler: Received Message from Resident");
	int priority = 2; // default
	switch(msg.event_type) {
		case EVENT_TRIGGER_EVENT_TYPE_VISITOR:
			priority = 2;
			break;
		case EVENT_TRIGGER_EVENT_TYPE_ASSISTANT:
			priority = 2;
			break;
	}
	ROS_INFO("Scheduler: Adding request to queue");
	eventQueue.push(EventNode(priority, msg));
}

void eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
	
	if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_RESPONSE) {
		if(msg.result == EVENT_TRIGGER_RESULT_SUCCESS){
			if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_VISITOR) {
				ROS_INFO("Scheduler: Response from Visitor");
			} else if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_ASSISTANT) {
				ROS_INFO("Scheduler: Response from Assitant");
			}
			// reset ability to send
			readyToSend = true;
		}
	}
}

int main(int argc, char **argv) {

	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Scheduler");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle nodeHandle;

	// advertise to event_trigger topic
	ros::Publisher eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);

	// subscribe to event_trigger topic
	ros::Subscriber eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, eventTriggerCallback);
	ros::Subscriber residentEventSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("resident_event",1000, residentEventCallback);
	
	ros::Rate loop_rate(10);
	
	ROS_INFO("Initializing Scheduler");

	//a count of howmany messages we have sent
	int count = 0;

	while (ros::ok()) {

		if (readyToSend) {

			if(eventQueue.size() > 0) {

				// block scheduler
				readyToSend = false;

				elderly_care_simulation::EventTrigger msg;
				msg = eventQueue.top().getEventTriggerMessage();
				eventQueue.pop();

				if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_VISITOR) {
					ROS_INFO("Scheduler: Publishing to Visitor");
				} else if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_ASSISTANT) {
					ROS_INFO("Scheduler: Publishing to Assitant");
				}
				eventTriggerPub.publish(msg);
			}
			
			
		}

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
