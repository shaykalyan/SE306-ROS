#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>

#include "StaticPoiConstants.h"
#include "EventTriggerUtility.h"
#include "PerformTaskConstants.h"
#include "ChefRobot.h"
#include "Robot.h"
#include "Poi.h"
#include "StaticPoi.h"
#include "EventNode.h"
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

ChefRobot chefRobot;

class ChefRobotTest : public ::testing::Test {
        
    protected:
        
        virtual void SetUp() {
            receivedEventTriggers.clear();
            chefRobot.currentLocationState = ChefRobot::AT_BASE;          
        }

};

/**
 * Tests that the ChefRobot responds correctly to Cooking events
 */
TEST_F(ChefRobotTest, chefRespondsToCookingRequests) {

    // Send relevant message to Chef
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_COOK;
    eventTriggerPub.publish(msg);
    
    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    ASSERT_EQ(ChefRobot::GOING_TO_STOVE, chefRobot.currentLocationState);
}

/**
 * Tests that the ChefRobot does not respond to non-Cooking events
 */
TEST_F(ChefRobotTest, chefDoesNotRespondToNonCookingRequests) {
    
    // Send irrelevant message to ChefRobot
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

    ASSERT_EQ(ChefRobot::AT_BASE, chefRobot.currentLocationState);
}

/**
 * Tests that the ChefRobot sends the correct event reply
 */
TEST_F(ChefRobotTest, friendSendsCorrectEventReply) {
    
    chefRobot.eventFinished();

    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    elderly_care_simulation::EventTrigger msg = receivedEventTriggers[0];
    ASSERT_EQ(EVENT_TRIGGER_MSG_TYPE_RESPONSE, msg.msg_type);
    ASSERT_EQ(EVENT_TRIGGER_EVENT_TYPE_COOK, msg.event_type);
    ASSERT_EQ(EVENT_TRIGGER_RESULT_SUCCESS, msg.result);
}

// Used to wait for msg is received
void eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    receivedEventTriggers.push_back(msg);
}

// Function wrapper for method
void friendEventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    chefRobot.eventTriggered(msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "ChefRobot");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);
    
    eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);
    eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, eventTriggerCallback);
    
    chefRobot.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);
    chefRobot.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, friendEventTriggerCallback);
    
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
