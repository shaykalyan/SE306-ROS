#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>

#include "../StaticPoiConstants.h"
#include "../EventTriggerUtility.h"
#include "../PerformTaskConstants.h"
#include "../EntertainmentRobot.h"
#include "../Robot.h"
#include "../Poi.h"
#include "../StaticPoi.h"
#include "../EventNode.h"
#include "elderly_care_simulation/EventTrigger.h"

using namespace elderly_care_simulation;

#include <unistd.h>

#include "gtest/gtest.h"

// Publishers
ros::Publisher eventTriggerPub;

// Subscribers
ros::Subscriber eventTriggerSub;

// Service Clients
ros::ServiceClient performTaskClient;

// Store received messages
std::vector<EventTrigger> receivedEventTriggers;

EntertainmentRobot entertainmentRobot;

class EntertainmentRobotTest : public ::testing::Test {
        
    protected:
        
        virtual void SetUp() {
            receivedEventTriggers.clear();
            entertainmentRobot.currentLocationState = EntertainmentRobot::AT_HOME;          
        }

};

/**
 * Tests that the EntertainmentRobot responds correctly to Entertainment events
 */
TEST_F(EntertainmentRobotTest, entertainmentRespondsToEntertainmentRequests) {

    // Send relevant message to Entertainment
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT;
    eventTriggerPub.publish(msg);
    
    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    ASSERT_EQ(EntertainmentRobot::GOING_TO_RESIDENT, entertainmentRobot.currentLocationState);
}

/**
 * Tests that the EntertainmentRobot does not respond to non-Entertainment events
 */
TEST_F(EntertainmentRobotTest, entertainmentDoesNotRespondToNonEntertainmentRequests) {
    
    // Send irrelevant message to EntertainmentRobot
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_FRIEND;
    eventTriggerPub.publish(msg);
    
    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    ASSERT_EQ(EntertainmentRobot::AT_HOME, entertainmentRobot.currentLocationState);
}

/**
 * Tests that the EntertainmentRobot sends the correct event reply
 */
TEST_F(EntertainmentRobotTest, friendSendsCorrectEventReply) {
    
    entertainmentRobot.eventTriggerReply();

    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    elderly_care_simulation::EventTrigger msg = receivedEventTriggers[0];
    ASSERT_EQ(EVENT_TRIGGER_MSG_TYPE_RESPONSE, msg.msg_type);
    ASSERT_EQ(EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT, msg.event_type);
    ASSERT_EQ(EVENT_TRIGGER_RESULT_SUCCESS, msg.result);
}

// Used to wait for msg is received
void eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    receivedEventTriggers.push_back(msg);
}

// Function wrapper for method
void friendEventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    entertainmentRobot.eventTriggerCallback(msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "EntertainmentRobot");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);
    
    eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);
    eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, eventTriggerCallback);
    
    entertainmentRobot.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);
    entertainmentRobot.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, friendEventTriggerCallback);
    
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
