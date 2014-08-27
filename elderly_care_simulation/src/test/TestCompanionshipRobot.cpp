#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>

#include "../StaticPoiConstants.h"
#include "../EventTriggerUtility.h"
#include "../PerformTaskConstants.h"
#include "../CompanionshipRobot.h"
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

CompanionshipRobot companionshipRobot;

class CompanionshipRobotTest : public ::testing::Test {
        
    protected:
        
        virtual void SetUp() {
            receivedEventTriggers.clear();
            companionshipRobot.currentLocationState = CompanionshipRobot::AT_HOME;          
        }

};

/**
 * Tests that the CompanionshipRobot responds correctly to Companionship events
 */
TEST_F(CompanionshipRobotTest, companionshipRespondsToCompanionshipRequests) {

    // Send relevant message to Companionship
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP;
    eventTriggerPub.publish(msg);
    
    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    ASSERT_EQ(CompanionshipRobot::GOING_TO_RESIDENT, companionshipRobot.currentLocationState);
}

/**
 * Tests that the CompanionshipRobot does not respond to non-Friend events
 */
TEST_F(CompanionshipRobotTest, companionshipDoesNotRespondToNonCompanionshipRequests) {
    
    // Send irrelevant message to CompanionshipRobot
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_EXERCISE;
    eventTriggerPub.publish(msg);
    
    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    ASSERT_EQ(CompanionshipRobot::AT_HOME, companionshipRobot.currentLocationState);
}

/**
 * Tests that the CompanionshipRobot sends the correct event reply
 */
TEST_F(CompanionshipRobotTest, friendSendsCorrectEventReply) {
    
    companionshipRobot.eventTriggerReply();

    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    elderly_care_simulation::EventTrigger msg = receivedEventTriggers[0];
    ASSERT_EQ(EVENT_TRIGGER_MSG_TYPE_RESPONSE, msg.msg_type);
    ASSERT_EQ(EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP, msg.event_type);
    ASSERT_EQ(EVENT_TRIGGER_RESULT_SUCCESS, msg.result);
}

// Used to wait for msg is received
void eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    receivedEventTriggers.push_back(msg);
}

// Function wrapper for method
void friendEventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    companionshipRobot.eventTriggerCallback(msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "TestCompanionshipRobot");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);
    
    eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);
    eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, eventTriggerCallback);
    
    companionshipRobot.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);
    companionshipRobot.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, friendEventTriggerCallback);
    
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}