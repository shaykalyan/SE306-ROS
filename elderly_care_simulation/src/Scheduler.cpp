#include "ros/ros.h"
#include "std_msgs/String.h"

#include "math.h"
#include "EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <queue>
#include "EventNode.h"
#include <unistd.h> // sleep


int ongoingEvents = 0;
const int MAX_CONCURRENT_EVENTS = 2;

// globals
std::priority_queue<EventNode > eventQueue;
ros::Publisher eventTriggerPub;
ros::Subscriber eventTriggerSub;

/**
 * 
 */
void residentEventCallback(elderly_care_simulation::EventTrigger msg) {
	ROS_INFO("Scheduler: Received Message from Resident");
	int priority = 2; // default
	switch(msg.event_type) {
		case EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT:
			priority = 2;
			break;
		case EVENT_TRIGGER_EVENT_TYPE_ILL:
			priority = 0;
			break;
        case EVENT_TRIGGER_EVENT_TYPE_VERY_ILL:
            priority = 0;
            break;
	}
	ROS_INFO("Scheduler: Adding request to queue");
	eventQueue.push(EventNode(priority, msg));
}

/**
 *
 */
void eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
	
	if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_RESPONSE) {
		if(msg.result == EVENT_TRIGGER_RESULT_SUCCESS){
			// reset ability to send
            if (msg.event_type != EVENT_TRIGGER_EVENT_TYPE_COOK) {
                ROS_INFO("Scheduler: Tasks reported done. Decreasing ongoingEvents");
                ongoingEvents--;
            }else{
                elderly_care_simulation::EventTrigger msg;
                msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
                msg.result = EVENT_TRIGGER_RESULT_FAILURE;
                msg.event_type = EVENT_TRIGGER_EVENT_TYPE_EAT;

                ROS_INFO("Scheduler: Adding EAT event to queue.");
                eventQueue.push(EventNode(1, msg));
            }
		}
	}
}


/**
 * Populates daily scheduled tasks into the event queue.
 * 
 * Daily Schedule will be queued in the following sequence:
 * NOTE: The timestamp is just for referencing purpose.
 *
 *  07:00   WAKE
 *  07:00   COOK ---> EAT
 *  07:00   MEDICATION
 *  08:00   EXERCISE
 *  09:00   SHOWER
 *  10:00   ENTERTAINMENT
 *  12:00   COOK ---> EAT
 *  12:00   MEDICATION
 *  13:00   CONVERSATION
 *  14:00   FRIEND & RELATIVE
 *  16:00   ENTERTAINMENT
 *  18:00   COOK ---> EAT
 *  18:00   MEDICATION
 *  19:00   COMPANIONSHIP
 *  20:00   SLEEP
 *  
 *  PAUSE FOR 20 SEC
 *  CLEAR LIST & REPOPULATE DAILY TASKS
 *
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

/**
 * Returns a string representation of the coresponding event type
 */
const char * eventTypeToString(int event_type) {
    switch(event_type){

        case EVENT_TRIGGER_EVENT_TYPE_EAT:              return "EAT";
        case EVENT_TRIGGER_EVENT_TYPE_SHOWER:           return "SHOWER";
        case EVENT_TRIGGER_EVENT_TYPE_EXERCISE:         return "EXERCISE";
        case EVENT_TRIGGER_EVENT_TYPE_CONVERSATION:     return "CONVERSATION";
        case EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT:    return "MORAL_SUPPORT";
        case EVENT_TRIGGER_EVENT_TYPE_FRIEND_RELATIVE:  return "FRIEND_RELATIVE";
        case EVENT_TRIGGER_EVENT_TYPE_ILL:              return "ILL";
        case EVENT_TRIGGER_EVENT_TYPE_VERY_ILL:         return "VERY_ILL";
        case EVENT_TRIGGER_EVENT_TYPE_MEDICATION:       return "MEDICATION";
        case EVENT_TRIGGER_EVENT_TYPE_COOK:             return "COOK";
        case EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT:    return "ENTERTAINMENT";
        case EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP:    return "COMPANIONSHIP";
        case EVENT_TRIGGER_EVENT_TYPE_WAKE:             return "WAKE";
        case EVENT_TRIGGER_EVENT_TYPE_SLEEP:            return "SLEEP";
    }
}

/**
 * Dequeues an event and publishes to the event_trigger topic.
 * Allows concurrent events to be triggered up to the limit
 * set in MAX_CONCURRENT_EVENTS
 *
 * COOK event is not affected as it acts independently of the 
 * Resident.
 */
void dequeueEvent(void) {
    if (eventQueue.size() < 1) {
        return;
    }

    elderly_care_simulation::EventTrigger msg;
    msg = eventQueue.top().getEventTriggerMessage();

    // cooking event type is published immediately
    if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_COOK) {
        eventQueue.pop();
        eventTriggerPub.publish(msg);
        ROS_INFO("Scheduler: Publishing event type: %s", eventTypeToString(msg.event_type));
    } else {
        // publish new event only if less than two events are ongoing
        if (ongoingEvents < MAX_CONCURRENT_EVENTS) {
            eventQueue.pop();
            eventTriggerPub.publish(msg);
            ROS_INFO("Scheduler: Publishing event type: %s", eventTypeToString(msg.event_type));
            ongoingEvents++;
        }       
    }
}


int main(int argc, char **argv) {

	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Scheduler");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle nodeHandle;

	// advertise to event_trigger topic
	eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);

	// subscribe to event_trigger topic
	eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, eventTriggerCallback);
	ros::Subscriber residentEventSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("resident_event",1000, residentEventCallback);
	
	ros::Rate loop_rate(10);

    // populate queue with day's events
    populateDailyTasks();

	//a count of howmany messages we have sent
	int count = 0;

	while (ros::ok()) {

        dequeueEvent(); 

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
