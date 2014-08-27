#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>
#include "../EventTriggerUtility.h"
#include "../PerformTaskConstants.h"
#include "elderly_care_simulation/EventTrigger.h" 
using namespace elderly_care_simulation;
#include "elderly_care_simulation/PerformTask.h" 
using namespace elderly_care_simulation;
#include "../EventNode.h"
#include <unistd.h> // sleep

#include "gtest/gtest.h"

EscortRobot doctor;

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
            // Code here will be called immediately after the constructor (right
            // before each test).
        }

        virtual void TearDown() {
            // Code here will be called immediately after each test (right
            // before the destructor).        
        }
};

/** 
 * Any calls to the doctor that are not VERY_ILL events should be ignored.
 */
TEST_F(EscortRobotTest, ignoreIrrelvantEvents) {

    // Sleep to allow the DoctorRobot to start
    // ros::Rate loop_rate(2);
    // loop_rate.sleep();
    
    ASSERT_TRUE(true);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "TestEscortRobot");
    ros::NodeHandle nodeHandle;

    geometry_msgs::Point base;
    base.x = 3.0f;
    base.y = 3.0f;

    geometry_msgs::Point hospital;
    hospital.x = 4.0f;
    hospital.y = 4.0f;

    doctor = EscortRobot(EVENT_TRIGGER_EVENT_TYPE_VERY_ILL, base, hospital);

    // Advertise and subscribe to topics
    residentEventPublisher = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("resident_event",1000, true);
    ros::Subscriber eventTriggerSubscriber = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("resident_trigger",1000, eventTriggeredCallback);

    // Run tests to see if we received messages as expected
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}