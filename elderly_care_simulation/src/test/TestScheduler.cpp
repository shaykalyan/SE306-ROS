#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>

#include "StaticPoiConstants.h"
#include "EventTriggerUtility.h"
#include "PerformTaskConstants.h"
#include "Scheduler.h"
#include "Poi.h"
#include "StaticPoi.h"
#include "elderly_care_simulation/EventTrigger.h"

using namespace elderly_care_simulation;

#include <unistd.h>

#include "gtest/gtest.h"

// Publishers
ros::Publisher eventTriggerPub;
ros::Publisher externalEventPub;

// Subscribers
ros::Subscriber eventTriggerSub;
ros::Subscriber externalEventSub;

// Store received messages
std::vector<EventTrigger> receivedEventTrigger;
std::vector<EventTrigger> receivedExternalEvent;

Scheduler scheduler;
EventTrigger msg;

class SchedulerTest : public ::testing::Test {
        
    protected:
        
        virtual void SetUp() {
            receivedEventTrigger = std::vector<EventTrigger>();         
        }

};

EventTrigger createEventRequestMsg(int eventType, int priority, int result){
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = eventType;
    msg.event_priority = priority;
    msg.event_weight = getEventWeight(eventType);
    msg.result = result;

    return msg;
}

EventTrigger createEventResponseMsg(int eventType, int result){
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;
    msg.event_type = eventType;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
    msg.event_weight = getEventWeight(eventType);
    msg.result = result;

    return msg;
}

void cleanUp() {
    receivedEventTrigger.clear();
    receivedExternalEvent.clear();
    scheduler.clearEventQueue();
    scheduler.resetConcurrentWeight();
    scheduler.resetRandomEventOccurrence();
}

void loopTillReceivedEventTriggerMessage(unsigned int number) {
    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTrigger.size() < number) {
        loop_rate.sleep();
        ros::spinOnce();
    }
}

void loopTillReceivedExternalEventMessage(unsigned int number) {
    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedExternalEvent.size() < number) {
        loop_rate.sleep();
        ros::spinOnce();
    }
}

bool isSameMessage(EventTrigger msg1, EventTrigger msg2) {
    return  msg1.msg_type == msg2.msg_type &&
            msg1.event_type == msg2.event_type &&
            msg1.event_priority == msg2.event_priority &&
            msg1.event_weight == msg2.event_weight &&
            msg1.result == msg2.result;
}

TEST_F(SchedulerTest, testNothing) {
    EXPECT_EQ(0, 0);
}

TEST_F(SchedulerTest, testSchedulerReceivingRequestInEventTrigger) {

    // Send REQUEST message to resident
    msg = createEventRequestMsg(2, 6, 2); // SHOWER, UNDEFINED, SUCCESS
    eventTriggerPub.publish(msg);
    
    loopTillReceivedEventTriggerMessage(1);

    if(isSameMessage(msg, receivedEventTrigger[0])) {
        EXPECT_EQ(0, scheduler.getEventQueueSize());
        EXPECT_EQ(0, scheduler.getConcurrentWeight());      
    } else {
        FAIL();
    }

    cleanUp();
}

TEST_F(SchedulerTest, testSchedulerReceivingFailureOrUndefinedInEventTrigger) {

    // TEST FAILURE
    msg = createEventResponseMsg(2, 1); // SHOWER, FAILURE
    eventTriggerPub.publish(msg);
    
    loopTillReceivedEventTriggerMessage(1);

    if(isSameMessage(msg, receivedEventTrigger[0])) {
        EXPECT_EQ(0, scheduler.getEventQueueSize());
        EXPECT_EQ(0, scheduler.getConcurrentWeight());      
    } else {
        FAIL();
    }

    cleanUp();

    // TEST UNDEFINED
    msg = createEventResponseMsg(2, 0); // SHOWER, UNDEFINED
    eventTriggerPub.publish(msg);
    
    loopTillReceivedEventTriggerMessage(1);

    if(isSameMessage(msg, receivedEventTrigger[0])) {
        EXPECT_EQ(0, scheduler.getEventQueueSize());
        EXPECT_EQ(0, scheduler.getConcurrentWeight());      
    } else {
        FAIL();
    }

    cleanUp();
}

TEST_F(SchedulerTest, testSchedulerReceivingEatResponseInEventTrigger) { 
     // TEST EAT
    msg = createEventResponseMsg(1, 2); // EAT, SUCCESS
    eventTriggerPub.publish(msg);
    
    loopTillReceivedEventTriggerMessage(1);

    if(isSameMessage(msg, receivedEventTrigger[0])) {
        EXPECT_EQ(0, scheduler.getEventQueueSize());
        EXPECT_EQ(-5, scheduler.getConcurrentWeight());      
    } else {
        FAIL();
    }

    cleanUp();
}

TEST_F(SchedulerTest, testSchedulerReceivingCookResponseInEventTrigger) {
    // TEST COOK
    msg = createEventResponseMsg(11, 2); // COOK, SUCCESS
    eventTriggerPub.publish(msg);
    
    loopTillReceivedEventTriggerMessage(1);

    if(isSameMessage(msg, receivedEventTrigger[0])) {
        EXPECT_EQ(1, scheduler.getEventQueueSize());
        EXPECT_EQ(0, scheduler.getConcurrentWeight());      
    } else {
        FAIL();
    }
    cleanUp();
}

TEST_F(SchedulerTest, testSchedulerReceivingShowerResponseInEventTrigger) {

    // TEST SHOWER
    msg = createEventResponseMsg(2, 2); // SHOWER, SUCCESS
    eventTriggerPub.publish(msg);
    
    loopTillReceivedEventTriggerMessage(1);

    if(isSameMessage(msg, receivedEventTrigger[0])) {
        EXPECT_EQ(0, scheduler.getEventQueueSize());
        EXPECT_EQ(-5, scheduler.getConcurrentWeight());      
    } else {
        FAIL();
    }

    cleanUp();
}

TEST_F(SchedulerTest, testSchedulerReceivingEntertainResponseInEventTrigger) {
    // TEST ENTERTAIN
    msg = createEventResponseMsg(12, 2); // ENTERTAIN, SUCCESS
    eventTriggerPub.publish(msg);
    
    loopTillReceivedEventTriggerMessage(1);

    if(isSameMessage(msg, receivedEventTrigger[0])) {
        EXPECT_EQ(0, scheduler.getEventQueueSize());
        EXPECT_EQ(-2, scheduler.getConcurrentWeight());      
    } else {
        FAIL();
    }

    cleanUp();
}


TEST_F(SchedulerTest, testSchedulerReceivingMoralSupportExternalEvent) {
    msg = createEventRequestMsg(5, 2, 0); // MORAL_SUPPORT, PRIORITY, SUCCESS
    externalEventPub.publish(msg);

    loopTillReceivedExternalEventMessage(1);

    if(isSameMessage(msg, receivedExternalEvent[0])) {
        EXPECT_EQ(1, scheduler.getEventQueueSize());
        EXPECT_EQ(0, scheduler.getConcurrentWeight());      
    } else {
        FAIL();
    }

    cleanUp();

    // Publish again, should still be the same size because
    // It only accept one MORAL_SUPPORT per day
    externalEventPub.publish(msg);
    externalEventPub.publish(msg);
    loopTillReceivedExternalEventMessage(2);

    if(isSameMessage(msg, receivedExternalEvent[0]) && isSameMessage(msg, receivedExternalEvent[1])) {
        EXPECT_EQ(1, scheduler.getEventQueueSize());
        EXPECT_EQ(0, scheduler.getConcurrentWeight());      
    } else {
        FAIL();
    }

    cleanUp();
}

TEST_F(SchedulerTest, testSchedulerReceivingEntertainmentExternalEvent) {
    msg = createEventRequestMsg(12, 0, 0); // ENTERTAIN, PRIORITY, SUCCESS
    externalEventPub.publish(msg);

    loopTillReceivedExternalEventMessage(1);

    if(isSameMessage(msg, receivedExternalEvent[0])) {
        EXPECT_EQ(1, scheduler.getEventQueueSize());
        EXPECT_EQ(0, scheduler.getConcurrentWeight());      
    } else {
        FAIL();
    }

    cleanUp();

    // Publish again, should increase the eventQueue size
    externalEventPub.publish(msg);
    externalEventPub.publish(msg);
    loopTillReceivedExternalEventMessage(2);

    if(isSameMessage(msg, receivedExternalEvent[0]) && isSameMessage(msg, receivedExternalEvent[1])) {
        EXPECT_EQ(2, scheduler.getEventQueueSize());
        EXPECT_EQ(0, scheduler.getConcurrentWeight());      
    } else {
        FAIL();
    }

    cleanUp();
}

// Used to wait for msg is received
void eventTriggerCallback(EventTrigger msg) {
    receivedEventTrigger.push_back(msg);
}

void externalEventReceivedCallback(EventTrigger msg) {
    receivedExternalEvent.push_back(msg);
}

// Function wrapper for method
void schedulerEventTriggerCallback(EventTrigger msg) {
    scheduler.eventTriggerCallback(msg);
}

// Used to wait for msg is received
void schedulerExternalEventReceivedCallback(EventTrigger msg) {
    scheduler.externalEventReceivedCallback(msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "TestScheduler");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);

    eventTriggerPub = nodeHandle.advertise<EventTrigger>("event_trigger",1000, true);
    eventTriggerSub = nodeHandle.subscribe<EventTrigger>("event_trigger",1000, eventTriggerCallback);
    externalEventPub = nodeHandle.advertise<EventTrigger>("external_event",1000, true);
    externalEventSub = nodeHandle.subscribe<EventTrigger>("external_event",1000, externalEventReceivedCallback);

    scheduler.eventTriggerPub = nodeHandle.advertise<EventTrigger>("event_trigger",1000, true);
    scheduler.eventTriggerSub = nodeHandle.subscribe<EventTrigger>("event_trigger",1000, schedulerEventTriggerCallback);
    scheduler.externalEventSub = nodeHandle.subscribe<EventTrigger>("external_event",1000, schedulerExternalEventReceivedCallback);
    

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}














    