#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include "math.h"
#include "EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <queue>
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


/*
    07:00   WAKE
    07:00   COOK ---> EAT
    07:00   MEDICATION
    08:00   EXERCISE
    09:00   SHOWER
    10:00   ENTERTAINMENT
    12:00   COOK ---> EAT
    12:00   MEDICATION
    13:00   CONVERSATION
    14:00   FRIEND & RELATIVE
    16:00   ENTERTAINMENT
    18:00   COOK ---> EAT
    18:00   MEDICATION
    19:00   COMPANIONSHIP
    20:00   SLEEP

    #PAUSE FOR 30 SEC
    #CLEAR LIST & REPOPULATE LIST
*/
void populateDailyTasks(void) {
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.result = EVENT_TRIGGER_RESULT_FAILURE;


    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_WAKE;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_COOK;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_MEDICATION;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_EXERCISE;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_SHOWER;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_COOK;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_MEDICATION;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_CONVERSATION;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_FRIEND_RELATIVE;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_COOK;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_MEDICATION;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP;
    eventQueue.push(EventNode(3, msg));

    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_SLEEP;
    eventQueue.push(EventNode(3, msg));

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

	//a count of howmany messages we have sent
	int count = 0;

	while (ros::ok()) {

		if (readyToSend) {

			if(eventQueue.size() > 0) {

				// block scheduler
				readyToSend = false;

                // dequeueEvent();

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
