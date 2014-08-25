#include "ros/ros.h"
#include "std_msgs/String.h"

#include "EventTriggerUtility.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <queue>
#include "EventNode.h"
#include <unistd.h> // sleep


// constants
const int MAX_CONCURRENT_WEIGHT = 2;

// globals
std::priority_queue<EventNode > eventQueue;
ros::Publisher eventTriggerPub;
ros::Subscriber eventTriggerSub;
ros::Subscriber randomEventSub;
int concurrentWeight = 0;

// ======================================
// =          SHOULD BE FALSE           =
// ======================================
bool allowNewEvents = false;
bool stopRosInfoSpam = false;

/**
 * Creates a Request EventTrigger message for requesting robot tasks.
 * @param eventType the event type for the message
 */
elderly_care_simulation::EventTrigger createEventRequestMsg(int eventType, int priority){
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = eventType;
    msg.event_priority = priority;
    msg.event_weight = getEventWeight(eventType);
    msg.result = EVENT_TRIGGER_RESULT_UNDEFINED;

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
        ROS_INFO("Scheduler: Additional events are not allowed at this time.");
        return;
    }

    ROS_INFO("Scheduler: Enqueuing event: [%s] with priority [%s]", 
             eventTypeToString(msg.event_type),
             priorityToString(msg.event_priority));

    eventQueue.push(EventNode(msg));
}

/**
 * Callback function to deal with events replied from service provider robots
 */
void eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_RESPONSE) {
        if(msg.result == EVENT_TRIGGER_RESULT_SUCCESS){

            if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_COOK) {
                ROS_INFO("Scheduler: [%s] done.", eventTypeToString(msg.event_type));

                elderly_care_simulation::EventTrigger eatMsg;
                eatMsg = createEventRequestMsg(EVENT_TRIGGER_EVENT_TYPE_EAT, EVENT_TRIGGER_PRIORITY_HIGH);
                ROS_INFO("Scheduler: Enqueuing event: [%s] with priority [%s]",
                          eventTypeToString(eatMsg.event_type),
                          priorityToString(eatMsg.event_priority));

                eventQueue.push(EventNode(eatMsg));
            }else{
                concurrentWeight -= msg.event_weight;
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

    int eventSequence[][2] = {

        // ======================================
        // =        COMMENTED OUT STUFF         =
        // ======================================

        // // Morning
        // { EVENT_TRIGGER_EVENT_TYPE_WAKE,            EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_COOK,            EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_MEDICATION,      EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_EXERCISE,        EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_SHOWER,          EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT,   EVENT_TRIGGER_PRIORITY_LOW },

        // // Noon
        // { EVENT_TRIGGER_EVENT_TYPE_COOK,            EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_MEDICATION,      EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_CONVERSATION,    EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_RELATIVE,        EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_FRIEND,          EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT,   EVENT_TRIGGER_PRIORITY_LOW },

        // // Evening
        // { EVENT_TRIGGER_EVENT_TYPE_COOK,            EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_MEDICATION,      EVENT_TRIGGER_PRIORITY_LOW },
         { EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP,   EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_SLEEP,           EVENT_TRIGGER_PRIORITY_VERY_LOW }
    };
    for(unsigned int i = 0; i < sizeof(eventSequence)/sizeof(*eventSequence); i++) {
        eventQueue.push(EventNode(createEventRequestMsg(eventSequence[i][0], eventSequence[i][1])));
    }
}

/**
 * Dequeues an event and publishes to the event_trigger topic.
 * Allows concurrent events to be triggered up to the limit
 * set in MAX_CONCURRENT_WEIGHT
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

    
    // Publish event if enough concurrent weight available
    if (concurrentWeight + msg.event_weight <= MAX_CONCURRENT_WEIGHT) {

        stopRosInfoSpam = false;
        switch(msg.event_type) {
            case EVENT_TRIGGER_EVENT_TYPE_SLEEP:
                allowNewEvents = false;

                eventQueue.pop();
                eventTriggerPub.publish(msg);
                ROS_INFO("Scheduler: Publishing event: [%s]", eventTypeToString(msg.event_type));
                concurrentWeight += msg.event_weight;
                break;

            case EVENT_TRIGGER_EVENT_TYPE_VERY_ILL:
                allowNewEvents = false;

                ROS_INFO("Scheduler: Publishing event: [%s]", eventTypeToString(msg.event_type));
                concurrentWeight += msg.event_weight;
                eventTriggerPub.publish(msg);

                clearEventQueue();

                // Enqueue a SLEEP event with max priority so when resident comes back from 
                // hospital it goes to sleep right away.
                // Since the queue is now empty after the SLEEP event, a new batch of daily
                // schedules will be repopulated automatically (in the main method).
                eventQueue.push(EventNode(createEventRequestMsg(EVENT_TRIGGER_EVENT_TYPE_SLEEP,
                                                                EVENT_TRIGGER_PRIORITY_VERY_HIGH)));
                break;

            default:
                allowNewEvents = true;
                eventTriggerPub.publish(msg);
                ROS_INFO("Scheduler: Publishing event: [%s]", eventTypeToString(msg.event_type));
                concurrentWeight += msg.event_weight;
                eventQueue.pop();
                break;
        }

    } else {
        if(!stopRosInfoSpam){
            ROS_INFO("Scheduler: Pending event: [%s]", 
                eventTypeToString(msg.event_type));
            stopRosInfoSpam = true;
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

    sleep(5);
    
    while (ros::ok()) {

        // ======================================
        // =        COMMENTED OUT STUFF         =
        // ======================================
        // if(eventQueue.size() == 0 && concurrentWeight == 0) {
        //     sleep(5);
        //     clearEventQueue();
        //     populateDailyTasks();
        // }else {
        //     dequeueEvent();
        // }
        dequeueEvent();

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
