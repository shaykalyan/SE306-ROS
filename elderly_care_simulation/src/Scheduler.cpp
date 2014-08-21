#include "ros/ros.h"
#include "std_msgs/String.h"

#include "EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <queue>
#include "EventNode.h"
#include <unistd.h> // sleep


// constants
const int MAX_CONCURRENT_EVENTS = 2;

const int VERY_HIGH_PRIORITY = 0;   // for emergency and custom
const int HIGH_PRIORITY = 1;        // for eat
const int MEDIUM_PRIORITY = 2;      // for random
const int LOW_PRIORITY = 3;         // for scheduled
const int VERY_LOW_PRIORITY = 4;    // for sleep

// globals
std::priority_queue<EventNode > eventQueue;
ros::Publisher eventTriggerPub;
ros::Subscriber eventTriggerSub;
ros::Subscriber randomEventSub;
int ongoingEvents = 0;
bool allowNewEvents = false;

/**
 * Returns a C string representation of the coresponding event type
 */
const char * eventTypeToString(int eventType) {
    switch(eventType){

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
        default:                                        return "-ERROR-";
    }
}

/**
 * Creates a Request EventTrigger message for requesting robot tasks.
 * @param eventType the event type for the message
 */
elderly_care_simulation::EventTrigger createEventRequestMsg(int eventType){
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = eventType;
    msg.result = EVENT_TRIGGER_RESULT_FAILURE;

    return msg;
}

/**
 * Clears the eventQueue.
 */
void clearEventQueue() {
    eventQueue = std::priority_queue<EventNode >();
}

/**
 * Callback function to deal with random events published by the resident
 */
void randomEventReceivedCallback(elderly_care_simulation::EventTrigger msg) {

    // Only allows random events to be added to event queue in the allowed
    // timeframe (between WAKE and SLEEP)
    if(!allowNewEvents) {
        ROS_INFO("Scheduler: Random events are not allowed at this time.");
        return;
    }

    int priority = MEDIUM_PRIORITY; // default
    switch(msg.event_type) {
        case EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT:
            priority = MEDIUM_PRIORITY;
            break;
        case EVENT_TRIGGER_EVENT_TYPE_ILL:
            priority = VERY_HIGH_PRIORITY;
            break;
        case EVENT_TRIGGER_EVENT_TYPE_VERY_ILL:
            priority = VERY_HIGH_PRIORITY;
            break;
    }

    ROS_INFO("Scheduler: Adding random event: [%s] to queue.", eventTypeToString(msg.event_type));
    eventQueue.push(EventNode(priority, msg));
}

/**
 * Callback function to deal with events replied from service provider robots
 */
void eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_RESPONSE) {
        if(msg.result == EVENT_TRIGGER_RESULT_SUCCESS){

            if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_COOK) {
                elderly_care_simulation::EventTrigger msg;
                msg = createEventRequestMsg(EVENT_TRIGGER_EVENT_TYPE_EAT);

                ROS_INFO("Scheduler: Adding [%s] event to queue.", eventTypeToString(msg.event_type));
                eventQueue.push(EventNode(1, msg));
            }else{
                ongoingEvents--;
                ROS_INFO("Scheduler: [%s] done.", eventTypeToString(msg.event_type));       
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
 *
 *  12:00   COOK ---> EAT
 *  12:00   MEDICATION
 *  13:00   CONVERSATION
 *  14:00   FRIEND & RELATIVE
 *  16:00   ENTERTAINMENT
 *
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

    int eventSequence[] = {

        // Morning
        EVENT_TRIGGER_EVENT_TYPE_WAKE,
        EVENT_TRIGGER_EVENT_TYPE_COOK,
        EVENT_TRIGGER_EVENT_TYPE_MEDICATION,
        EVENT_TRIGGER_EVENT_TYPE_EXERCISE,
        EVENT_TRIGGER_EVENT_TYPE_SHOWER,
        EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT,

        // Noon
        EVENT_TRIGGER_EVENT_TYPE_COOK,
        EVENT_TRIGGER_EVENT_TYPE_MEDICATION,
        EVENT_TRIGGER_EVENT_TYPE_CONVERSATION,
        EVENT_TRIGGER_EVENT_TYPE_FRIEND_RELATIVE,
        EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT,

        // Evening
        EVENT_TRIGGER_EVENT_TYPE_COOK,
        EVENT_TRIGGER_EVENT_TYPE_MEDICATION,
        EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP,
        EVENT_TRIGGER_EVENT_TYPE_SLEEP
    };

    for(unsigned int i = 0; i < sizeof(eventSequence)/sizeof(*eventSequence); i++) {
        switch(eventSequence[i]) {
            case EVENT_TRIGGER_EVENT_TYPE_SLEEP:
                eventQueue.push(EventNode(VERY_LOW_PRIORITY, createEventRequestMsg(eventSequence[i])));
                break;
            default:
                eventQueue.push(EventNode(LOW_PRIORITY, createEventRequestMsg(eventSequence[i])));
                break;
        }
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

    // Enable random events to be added to queue only after WAKE event
    if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_WAKE){
        allowNewEvents = true;
    }

    // Disable random events from being added to queue after SLEEP event
    // Also prevent the SLEEP event until all tasks are finished.
    if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_SLEEP){
        allowNewEvents = false;

        // If no more tasks, then publish SLEEP event
        if(ongoingEvents == 0) {
            eventQueue.pop();
            eventTriggerPub.publish(msg);
            ROS_INFO("Scheduler: Publishing event: [%s]", eventTypeToString(msg.event_type));
            ongoingEvents++;

        // If tasks still ongoing, do not allow SLEEP
        }else{
            ROS_INFO("Scheduler: Target still busy, cannot do event: [%s]", eventTypeToString(msg.event_type));
            return;
        }

    } else if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_VERY_ILL) {
        ROS_INFO("Scheduler: Publishing event: [%s]", eventTypeToString(msg.event_type));
        ongoingEvents++;
        eventTriggerPub.publish(msg);

        allowNewEvents = false;

        // Pop everything except sleep, which will be executed after the
        // Resident comes back from hospital
        while(eventQueue.top().getEventTriggerMessage().event_type != EVENT_TRIGGER_EVENT_TYPE_SLEEP) {
            eventQueue.pop();
        }

    // cooking event type is published immediately
    } else if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_COOK) {
        eventQueue.pop();
        eventTriggerPub.publish(msg);
        ROS_INFO("Scheduler: Publishing event: [%s]", eventTypeToString(msg.event_type));
    } else {
        // publish new event only if less than two events are ongoing
        if (ongoingEvents < MAX_CONCURRENT_EVENTS) {
            eventQueue.pop();
            eventTriggerPub.publish(msg);
            ROS_INFO("Scheduler: Publishing event: [%s]", eventTypeToString(msg.event_type));
            ongoingEvents++;
        }       
    }
}

/**
 * Main
 */
int main(int argc, char **argv) {

    //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
    ros::init(argc, argv, "Scheduler");

    //NodeHandle is the main access point to communicate with ros.
    ros::NodeHandle nodeHandle;

    // advertise to event_trigger topic
    eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);

    // subscribe to event_trigger topic
    eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, eventTriggerCallback);
    randomEventSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("resident_event",1000, randomEventReceivedCallback);
    
    ros::Rate loop_rate(10);

    // populate queue with day's events
    populateDailyTasks();

    //a count of howmany messages we have sent
    int count = 0;

    while (ros::ok()) {

        if(eventQueue.size() == 0 && ongoingEvents == 0) {
            sleep(5);
            clearEventQueue();
            populateDailyTasks();
        }else {
            dequeueEvent();
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
