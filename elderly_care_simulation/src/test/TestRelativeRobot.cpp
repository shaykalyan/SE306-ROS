#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>

#include "../StaticPoiConstants.h"
#include "../EventTriggerUtility.h"
#include "../PerformTaskConstants.h"
#include "../RelativeRobot.h"
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

RelativeRobot relative;

class RelativeRobotTest : public ::testing::Test {
        
    protected:
        
        virtual void SetUp() {
            receivedEventTriggers.clear();
            relative.currentLocationState = RelativeRobot::AT_HOME;          
        }

};

/**
 * Tests that the RelativeRobot responds correctly to Relative events
 */
TEST_F(RelativeRobotTest, relativeRespondsToRelativeRequests) {

    // Send relevant message to RelativeRobot
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_RELATIVE;
    eventTriggerPub.publish(msg);
    
    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    ASSERT_EQ(RelativeRobot::GOING_TO_RESIDENT, relative.currentLocationState);
}

/**
 * Tests that the RelativeRobot does not respond to non-Relative events
 */
TEST_F(RelativeRobotTest, relativeDoesNotRespondToNonRelativeRequests) {
    
    // Send irrelevant message to RelativeRobot
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

    ASSERT_EQ(RelativeRobot::AT_HOME, relative.currentLocationState);
}

/**
 * Tests that the RelativeRobot sends the correct event reply
 */
TEST_F(RelativeRobotTest, relativeSendsCorrectEventReply) {
    
    relative.eventTriggerReply();

    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    elderly_care_simulation::EventTrigger msg = receivedEventTriggers[0];
    ASSERT_EQ(EVENT_TRIGGER_MSG_TYPE_RESPONSE, msg.msg_type);
    ASSERT_EQ(EVENT_TRIGGER_EVENT_TYPE_RELATIVE, msg.event_type);
    ASSERT_EQ(EVENT_TRIGGER_RESULT_SUCCESS, msg.result);
}

// Used to wait for msg is received
void eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    receivedEventTriggers.push_back(msg);
}

// Function wrapper for method
void relativeEventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    relative.eventTriggerCallback(msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "TestRelativeRobot");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);
    
    eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);
    eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, eventTriggerCallback);
    
    relative.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);
    relative.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, relativeEventTriggerCallback);
    
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}














    
