#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>

#include "../StaticPoiConstants.h"
#include "../EventTriggerUtility.h"
#include "../PerformTaskConstants.h"
#include "../Caregiver.h"
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

Caregiver theCaregiver;

class CaregiverRobotTest : public ::testing::Test {
        
    protected:
        
        virtual void SetUp() {
            receivedEventTriggers.clear();
            theCaregiver.currentLocationState = Caregiver::AT_HOME;          
        }

};

/**
 * Tests that the Caregiver responds correctly to Exercise events
 */
TEST_F(CaregiverRobotTest, caregiverRespondsToExerciseRequests) {

    // Send relevant message to Caregiver
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

    ASSERT_EQ(Caregiver::GOING_TO_RESIDENT, theCaregiver.currentLocationState);
}

/**
 * Tests that the Caregiver responds correctly to Shower events
 */
TEST_F(CaregiverRobotTest, caregiverRespondsToShowerRequests) {

    // Send relevant message to Caregiver
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_SHOWER;
    eventTriggerPub.publish(msg);
    
    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    ASSERT_EQ(Caregiver::GOING_TO_RESIDENT, theCaregiver.currentLocationState);
}

/**
 * Tests that the Caregiver responds correctly to Conversation events
 */
TEST_F(CaregiverRobotTest, caregiverRespondsToConversationRequests) {

    // Send relevant message to Caregiver
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_CONVERSATION;
    eventTriggerPub.publish(msg);
    
    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    ASSERT_EQ(Caregiver::GOING_TO_RESIDENT, theCaregiver.currentLocationState);
}

// *
//  * Tests that the Caregiver responds correctly to Moral Support events
 
TEST_F(CaregiverRobotTest, caregiverRespondsToMoralSupportRequests) {

    // Send relevant message to Caregiver
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT;
    eventTriggerPub.publish(msg);
    
    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    ASSERT_EQ(Caregiver::GOING_TO_RESIDENT, theCaregiver.currentLocationState);
}

/**
 * Tests that the Caregiver does not respond to non-caregiver events
 */
TEST_F(CaregiverRobotTest,caregiverDoesNotRespondToNonExerciseRequests) {
    
    // Send irrelevant message to Caregiver
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

    ASSERT_EQ(Caregiver::AT_HOME, theCaregiver.currentLocationState);
}

/**
 *Tests that the Caregiver get the right reply for CONVERSATION event
**/

TEST_F(CaregiverRobotTest, caregiverSendsCorrectConversationEventReply) {
    theCaregiver.MY_TASK = EVENT_TRIGGER_EVENT_TYPE_CONVERSATION;
    theCaregiver.eventTriggerReply();

    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    elderly_care_simulation::EventTrigger msg = receivedEventTriggers[0];
    ASSERT_EQ(EVENT_TRIGGER_MSG_TYPE_RESPONSE, msg.msg_type);
    ASSERT_EQ(EVENT_TRIGGER_EVENT_TYPE_CONVERSATION, msg.event_type);
    ASSERT_EQ(EVENT_TRIGGER_RESULT_SUCCESS, msg.result);
}

/**
 *Tests that the Caregiver get the right reply for SHOWER event
**/

TEST_F(CaregiverRobotTest, caregiverSendsCorrectShowerEventReply) {
    theCaregiver.MY_TASK = EVENT_TRIGGER_EVENT_TYPE_SHOWER;
    theCaregiver.eventTriggerReply();

    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    elderly_care_simulation::EventTrigger msg = receivedEventTriggers[0];
    ASSERT_EQ(EVENT_TRIGGER_MSG_TYPE_RESPONSE, msg.msg_type);
    ASSERT_EQ(EVENT_TRIGGER_EVENT_TYPE_SHOWER, msg.event_type);
    ASSERT_EQ(EVENT_TRIGGER_RESULT_SUCCESS, msg.result);
}

/**
 *Tests that the Caregiver get the right reply for EXERCISE event
**/

TEST_F(CaregiverRobotTest, caregiverSendsCorrecMoralSupportEventReply) {
    theCaregiver.MY_TASK = EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT;
    theCaregiver.eventTriggerReply();

    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    elderly_care_simulation::EventTrigger msg = receivedEventTriggers[0];
    ASSERT_EQ(EVENT_TRIGGER_MSG_TYPE_RESPONSE, msg.msg_type);
    ASSERT_EQ(EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT, msg.event_type);
    ASSERT_EQ(EVENT_TRIGGER_RESULT_SUCCESS, msg.result);
}

/**
 *Tests that the Caregiver get the right reply for EXERCISE event
**/

TEST_F(CaregiverRobotTest, caregiverSendsCorrectExerciseEventReply) {
    theCaregiver.MY_TASK = EVENT_TRIGGER_EVENT_TYPE_EXERCISE;
    theCaregiver.eventTriggerReply();

    // Wait until message is received
    ros::Rate loop_rate(10);
    while (receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    elderly_care_simulation::EventTrigger msg = receivedEventTriggers[0];
    ASSERT_EQ(EVENT_TRIGGER_MSG_TYPE_RESPONSE, msg.msg_type);
    ASSERT_EQ(EVENT_TRIGGER_EVENT_TYPE_EXERCISE, msg.event_type);
    ASSERT_EQ(EVENT_TRIGGER_RESULT_SUCCESS, msg.result);
}

// Used to wait for msg is received
void eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    receivedEventTriggers.push_back(msg);
}

// Function wrapper for method
void caregiverEventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    theCaregiver.eventTriggerCallback(msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "TestCaregiverRobot");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);
    eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);
    eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, eventTriggerCallback);
    
    theCaregiver.eventTriggerPub = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);
    theCaregiver.eventTriggerSub = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, caregiverEventTriggerCallback);
    
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}














    
