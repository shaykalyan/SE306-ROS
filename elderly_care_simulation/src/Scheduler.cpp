#include "ros/ros.h"

#include "EventTriggerUtility.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <queue>
#include <unistd.h> // sleep
#include "Scheduler.h"

using namespace elderly_care_simulation;

Scheduler scheduler;

Scheduler::Scheduler(){
    concurrentWeight = 0;
    allowNewEvents = true;
    stopRosInfoSpam = false;

    randomEventLimit[EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT] = false;
    randomEventLimit[EVENT_TRIGGER_EVENT_TYPE_ILL] = false;
    randomEventLimit[EVENT_TRIGGER_EVENT_TYPE_VERY_ILL] = false;
}

Scheduler::~Scheduler() {}

/**
 * Creates a Request EventTrigger message for requesting robot tasks.
 * @param eventType the event type for the message
 */
EventTrigger Scheduler::createEventRequestMsg(int eventType, int priority){
    EventTrigger msg;
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
void Scheduler::clearEventQueue() {
    eventQueue = std::priority_queue<EventNode >();
}

/**
 * Reset all random event occurrence back to false.
 */
void Scheduler::resetRandomEventOccurrence() {

    for(std::map<int, bool >::iterator iter = randomEventLimit.begin(); iter != randomEventLimit.end(); ++iter) {
        randomEventLimit[iter->first] = false;
    }
}

void Scheduler::resetConcurrentWeight() {
    concurrentWeight = 0;
}

/**
 * Returns the current concurrent weight count
 */
int Scheduler::getConcurrentWeight() const {
    return concurrentWeight;
}


/**
 * Returns the current event queue size
 */
int Scheduler::getEventQueueSize() const {
    return eventQueue.size();
}

/**
 * Callback function to deal with external events
 */
void Scheduler::externalEventReceivedCallback(EventTrigger msg) {

    // Only allows random events to be added to event queue in the allowed
    // timeframe (between WAKE and SLEEP)
    if(!allowNewEvents) {
        ROS_INFO("Scheduler: Additional events are not allowed at this time.");
        return;
    }

    // If the incoming event is a random event
    if(randomEventLimit.count(msg.event_type) != 0) {

        // If it havent occured before, change the flag and continue
        if(randomEventLimit[msg.event_type] == false) {
            randomEventLimit[msg.event_type] = true;

        // If event occured before, then block it
        } else {
            ROS_INFO("Scheduler: [%s] cannot occur more than once per day.", 
                eventTypeToString(msg.event_type));
            return;
        }
    }

    ROS_INFO("Scheduler: Enqueuing event: [%s] with priority [%s]", 
             eventTypeToString(msg.event_type),
             priorityToString(msg.event_priority));

    eventQueue.push(EventNode(msg));
}

/**
 * Callback function to deal with events replied from service provider robots
 */
void Scheduler::eventTriggerCallback(EventTrigger msg) {
    
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_RESPONSE) {
        if(msg.result == EVENT_TRIGGER_RESULT_SUCCESS){

            if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_COOK) {
                ROS_INFO("Scheduler: [%s] done.", eventTypeToString(msg.event_type));

                EventTrigger eatMsg;
                eatMsg = createEventRequestMsg(EVENT_TRIGGER_EVENT_TYPE_EAT, EVENT_TRIGGER_PRIORITY_HIGH);
                ROS_INFO("Scheduler: Enqueuing event: [%s] with priority [%s]",
                          eventTypeToString(eatMsg.event_type),
                          priorityToString(eatMsg.event_priority));

                eventQueue.push(EventNode(eatMsg));
            }else{

                // ILL has a weight of 0, but still blocks all other events (taking up 2 slots)
                // Therefore need to -2 to concurrent weight to free the slot.
                if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_ILL ||
                    msg.event_type == EVENT_TRIGGER_EVENT_TYPE_VERY_ILL) {
                    concurrentWeight -= 2;
                }else {
                    concurrentWeight -= msg.event_weight;
                }
                
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
void Scheduler::populateDailyTasks() {

    int eventSequence[][2] = {

        // ======================================
        // =        COMMENTED OUT STUFF         =
        // ======================================

        // // Morning
        // { EVENT_TRIGGER_EVENT_TYPE_WAKE,            EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_COOK,            EVENT_TRIGGER_PRIORITY_LOW }
        // { EVENT_TRIGGER_EVENT_TYPE_MOVE_TO_HALLWAY, EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_MEDICATION,      EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_EXERCISE,        EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_SHOWER,          EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_MOVE_TO_BEDROOM, EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_COOK,            EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT,   EVENT_TRIGGER_PRIORITY_LOW },

        // // Noon
        // { EVENT_TRIGGER_EVENT_TYPE_MEDICATION,      EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_CONVERSATION,    EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_MOVE_TO_HALLWAY, EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_RELATIVE,        EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_FRIEND,          EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_COOK,            EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT,   EVENT_TRIGGER_PRIORITY_LOW },


        // // Evening
        // { EVENT_TRIGGER_EVENT_TYPE_MEDICATION,      EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_MOVE_TO_BEDROOM, EVENT_TRIGGER_PRIORITY_LOW },
        // { EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP,   EVENT_TRIGGER_PRIORITY_LOW },
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
void Scheduler::dequeueEvent() {

    if (eventQueue.size() < 1) {
        return;
    }

    EventTrigger msg;
    msg = eventQueue.top().getEventTriggerMessage();

    
    // Publish event if enough concurrent weight available
    if (concurrentWeight + msg.event_weight <= MAX_CONCURRENT_WEIGHT) {

        ROS_INFO("Scheduler: Publishing event: [%s]", eventTypeToString(msg.event_type));

        stopRosInfoSpam = false;
        switch(msg.event_type) {
            case EVENT_TRIGGER_EVENT_TYPE_SLEEP:
                allowNewEvents = true;

                concurrentWeight += msg.event_weight;
                eventTriggerPub.publish(msg);
                eventQueue.pop();
                
                break;

            case EVENT_TRIGGER_EVENT_TYPE_ILL:
                allowNewEvents = false;

                // ILL has a weight of 0, but still blocks all other events
                concurrentWeight += 2;
                eventTriggerPub.publish(msg);
                eventQueue.pop();

                eventQueue.push(EventNode(createEventRequestMsg(EVENT_TRIGGER_EVENT_TYPE_SLEEP,
                                                                EVENT_TRIGGER_PRIORITY_VERY_HIGH)));
                eventQueue.push(EventNode(createEventRequestMsg(EVENT_TRIGGER_EVENT_TYPE_WAKE,
                                                                EVENT_TRIGGER_PRIORITY_VERY_HIGH)));
                break;

            case EVENT_TRIGGER_EVENT_TYPE_VERY_ILL:
                allowNewEvents = false;

                // VERY_ILL has a weight of 0, but still blocks all other events
                concurrentWeight += 2;
                eventTriggerPub.publish(msg);
                eventQueue.pop();

                clearEventQueue();

                // Enqueue a SLEEP event with max priority so when resident comes back from 
                // hospital it goes to sleep right away.
                // Since the queue is now empty after the SLEEP event, a new batch of daily
                // schedules will be repopulated automatically (in the main method).
                eventQueue.push(EventNode(createEventRequestMsg(EVENT_TRIGGER_EVENT_TYPE_SLEEP,
                                                                EVENT_TRIGGER_PRIORITY_VERY_HIGH)));
                break;

            default:
                if(msg.event_type == EVENT_TRIGGER_EVENT_TYPE_WAKE) {
                    allowNewEvents = true;
                }
                
                eventTriggerPub.publish(msg);
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

void callEventTriggerCallback(EventTrigger msg) {
    scheduler.eventTriggerCallback(msg);
}

void callExternalEventReceivedCallback(EventTrigger msg) {
    scheduler.externalEventReceivedCallback(msg);
}

/**
 * Main
 */
int main(int argc, char **argv) {

    //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
    ros::init(argc, argv, "Scheduler");

    //NodeHandle is the main access point to communicate with ros.
    ros::NodeHandle nodeHandle;

    scheduler = Scheduler();

    // advertise to event_trigger topic
    scheduler.eventTriggerPub = nodeHandle.advertise<EventTrigger>("event_trigger",1000, true);

    // subscribe to event_trigger topic
    scheduler.eventTriggerSub = nodeHandle.subscribe<EventTrigger>("event_trigger",1000, callEventTriggerCallback);
    scheduler.externalEventSub = nodeHandle.subscribe<EventTrigger>("external_event",1000, callExternalEventReceivedCallback);
    
    ros::Rate loop_rate(10);

    // populate queue with day's events
    
    scheduler.populateDailyTasks();

    //a count of howmany messages we have sent
    int count = 0;
    sleep(5);

    sleep(1);

    while (ros::ok()) {

        if(scheduler.getEventQueueSize() == 0 && scheduler.getConcurrentWeight() <= 0) {
            ROS_INFO("Day Ends....");
            sleep(10);
            scheduler.clearEventQueue();
            scheduler.resetConcurrentWeight();
            scheduler.resetRandomEventOccurrence();
            scheduler.populateDailyTasks();
            ROS_INFO("Day Starts....");
        }else {
            scheduler.dequeueEvent();
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
