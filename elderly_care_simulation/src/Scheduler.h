#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "ros/ros.h"

#include "EventTriggerUtility.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/GuiComm.h"
#include <queue>
#include "EventNode.h"
#include <unistd.h> // sleep

class Scheduler {

static const int MAX_CONCURRENT_WEIGHT = 2;

public:

    ros::Publisher eventTriggerPub;
    ros::Publisher guiCommPub;
    ros::Subscriber eventTriggerSub;
    ros::Subscriber externalEventSub;
    ros::Subscriber guiCommSub;

    Scheduler();
    ~Scheduler();
    elderly_care_simulation::EventTrigger createEventRequestMsg(int eventType, int priority);
    void externalEventReceivedCallback(elderly_care_simulation::EventTrigger msg);
    void eventTriggerCallback(elderly_care_simulation::EventTrigger msg);
    void guiCommunicationCallback(elderly_care_simulation::GuiComm msg);
    int getConcurrentWeight() const;
    int getEventQueueSize() const;
    void clearEventQueue();
    void resetRandomEventOccurrence();
    void resetConcurrentWeight();
    void populateDailyTasks();
    void dequeueEvent();

private:

    int concurrentWeight;
    std::priority_queue<EventNode > eventQueue;
    std::map<int, bool> randomEventLimit;
    bool allowNewEvents;
    bool stopRosInfoSpam;
};

#endif