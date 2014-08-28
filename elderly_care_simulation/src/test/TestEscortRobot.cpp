#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>
#include "EventTriggerUtility.h"
#include "PerformTaskConstants.h"
#include "elderly_care_simulation/EventTrigger.h" 
using namespace elderly_care_simulation;
#include "EventNode.h"
#include <unistd.h> // sleep

#define private public
#include "EscortRobot.h"

#include "gtest/gtest.h"

EscortRobot doctor;

// Store received messages
std::vector<EventTrigger> receivedEventTriggers;

// Publishers and subcribers
//ros::Publisher residentEventPublisher;
ros::Subscriber residentEventSubscriber;

/**
 * Callback for the 'event_trigger' topic messages.
 * Simply logs the received message into a list.
 */
void eventTriggerCallback(EventTrigger msg) {
    receivedEventTriggers.push_back(msg);
}

//void eventTriggeredCallback(EventTrigger msg) {
//    doctor.eventTriggered(msg);
//}


/**
 * This test suite tests the functionality of the EscortRobot class.
 * The EscortRobot ROS node under test is DoctorRobot.
 */
class EscortRobotTest : public ::testing::Test {
    protected:
        // You can remove any or all of the following functions if its body
        // is empty.

        EscortRobotTest() {
            // You can put any initialisation here.
        }

        virtual ~EscortRobotTest() {
            // You can do clean-up work that doesn't throw exceptions here.
        }

        // If the constructor and destructor are not enough for setting up
        // and cleaning up each test, you can define the following methods:

        virtual void SetUp() {
            // Clear any received messages
            // Store received messages
            receivedEventTriggers.clear();

            doctor.performingTask = false;
            doctor.currentLocationState = EscortRobot::AT_BASE;
        }

        virtual void TearDown() {
            // Code here will be called immediately after each test (right
            // before the destructor).        
        }
};

EventTrigger eventTriggerForEventType(int eventType) {
    ROS_INFO("Making a message");
    EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg.event_type = eventType;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_HIGH;
    msg.event_weight = getEventWeight(eventType);
    msg.result = EVENT_TRIGGER_RESULT_UNDEFINED;
    return msg;
}

/** 
 * Any calls to the doctor that are not VERY_ILL events should be ignored.
 */
TEST_F(EscortRobotTest, ignoreIrrelevantEvents) {
    ROS_INFO("Starting test");

    ASSERT_EQ(EscortRobot::AT_BASE, doctor.currentLocationState);
    ASSERT_FALSE(doctor.performingTask);

    doctor.eventTriggered(eventTriggerForEventType(EVENT_TRIGGER_EVENT_TYPE_MEDICATION));

    // The doctor should not change its state
    ASSERT_EQ(EscortRobot::AT_BASE, doctor.currentLocationState);
    ASSERT_FALSE(doctor.performingTask);
}

/** 
 * Any calls to the doctor that are VERY_ILL events should result in a change
 * of state in the doctor.
 */
TEST_F(EscortRobotTest, acceptRelevantEvents) {

    ASSERT_EQ(EscortRobot::AT_BASE, doctor.currentLocationState);
    ASSERT_FALSE(doctor.performingTask);

    doctor.eventTriggered(eventTriggerForEventType(EVENT_TRIGGER_EVENT_TYPE_VERY_ILL));

    // The doctor should change its state
    //ASSERT_GT(doctor.locationQueue.size(), 0);
    ASSERT_TRUE(doctor.performingTask);
    ASSERT_EQ(EscortRobot::GOING_TO_POI, doctor.currentLocationState);
}

/** 
 * Asking the doctor to notify completion of its task should result in the correct
 * EventTrigger message being send on the "event_trigger" topic.
 */
TEST_F(EscortRobotTest, notifyTaskCompletion) {

    doctor.notifySchedulerOfTaskCompletion();

    // Wait until a message has been received
    ros::Rate loop_rate(10);
    while(receivedEventTriggers.size() == 0) {
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    /*** ASSERTIONS BEGIN ***/
    
    // Check we received only one EventTrigger
    ASSERT_EQ(1, receivedEventTriggers.size());
    
    EventTrigger msg = receivedEventTriggers[0];
    
    // Check that the message has the correct information
    ASSERT_EQ(EVENT_TRIGGER_MSG_TYPE_RESPONSE, msg.msg_type);
    ASSERT_EQ(EVENT_TRIGGER_EVENT_TYPE_VERY_ILL, msg.event_type);
    ASSERT_EQ(EVENT_TRIGGER_RESULT_SUCCESS, msg.result);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "TestEscortRobot");
    ros::NodeHandle nodeHandle;

    // Instantiate a new doctor for each test
    geometry_msgs::Point base;
    base.x = 3.0f;
    base.y = 3.0f;

    geometry_msgs::Point hospital;
    hospital.x = 4.0f;
    hospital.y = 4.0f;

    doctor = EscortRobot(EVENT_TRIGGER_EVENT_TYPE_VERY_ILL, base, hospital);

    // Advertise and subscribe to topics
    //residentEventPublisher = nodeHandle.advertise<EventTrigger>("event_trigger",1000, true);
    doctor.eventTriggerPub = nodeHandle.advertise<EventTrigger>("event_trigger",1000, true);
    ros::Subscriber eventTriggerSub = nodeHandle.subscribe<EventTrigger>("event_trigger", 1000, eventTriggerCallback);

    // Run tests to see if we received messages as expected
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}