#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>
#include "../EventTriggerUtility.h"
#include "../PerformTaskConstants.h"
#include "elderly_care_simulation/EventTrigger.h" 
using namespace elderly_care_simulation;
#include "../EventNode.h"
#include <unistd.h> // sleep

#define private public
#include "../EscortRobot.h"

#include "gtest/gtest.h"

EscortRobot doctor;

// Publishers and subcribers
ros::Publisher residentEventPublisher;
ros::Subscriber residentEventSubscriber;

void eventTriggeredCallback(elderly_care_simulation::EventTrigger msg) {
    doctor.eventTriggered(msg);
}


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
            // Instantiate a new doctor for each test
            geometry_msgs::Point base;
            base.x = 3.0f;
            base.y = 3.0f;

            geometry_msgs::Point hospital;
            hospital.x = 4.0f;
            hospital.y = 4.0f;

            doctor = EscortRobot(EVENT_TRIGGER_EVENT_TYPE_VERY_ILL, base, hospital);
        }

        virtual void TearDown() {
            // Code here will be called immediately after each test (right
            // before the destructor).        
        }
};

EventTrigger eventTriggerForEventType(int eventType) {
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
TEST_F(EscortRobotTest, ignoreIrrelvantEvents) {

    ASSERT_EQ(EscortRobot::AT_BASE, doctor.currentLocationState);
    ASSERT_FALSE(doctor.performingTask);

    residentEventPublisher.publish(eventTriggerForEventType(EVENT_TRIGGER_EVENT_TYPE_MEDICATION));

    // The doctor should not change its state
    ASSERT_EQ(EscortRobot::AT_BASE, doctor.currentLocationState);
    ASSERT_FALSE(doctor.performingTask);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "TestEscortRobot");
    ros::NodeHandle nodeHandle;

    // Advertise and subscribe to topics
    residentEventPublisher = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("resident_event",1000, true);
    residentEventSubscriber = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("resident_trigger",1000, eventTriggeredCallback);

    // Run tests to see if we received messages as expected
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}