#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>
#include "../EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h" 
using namespace elderly_care_simulation;
#include "../EventNode.h"
#include <unistd.h> // sleep

#include "gtest/gtest.h"


// Publishers
ros::Publisher residentEventPublisher;

// Subscribers
ros::Subscriber eventTriggerSubscriber;

// Store received messages
std::vector<EventTrigger> receivedEventTriggers;

/**
 * The fixture for testing the Scheduler.
 * 
 * Most of these methods could be removed, but they have been left as
 * an example.
 */
class SchedulerTest : public ::testing::Test {
	protected:
		// You can remove any or all of the following functions if its body
		// is empty.

		SchedulerTest() {
			// You can put any initialisation here.
		}

		virtual ~SchedulerTest() {
			// You can do clean-up work that doesn't throw exceptions here.
		}

		// If the constructor and destructor are not enough for setting up
		// and cleaning up each test, you can define the following methods:

		virtual void SetUp() {
			// Code here will be called immediately after the constructor (right
			// before each test).
			receivedEventTriggers.clear();
		}

		virtual void TearDown() {
			// Code here will be called immediately after each test (right
			// before the destructor).
		}
};

/** 
 * Sending an EventTrigger on topic 'resident_event' to the scheduler
 * should result in an EventTrigger published on 'event_trigger'
 */
TEST_F(SchedulerTest, requestAssistant) {
	
	// Send message to the scheduler
	EventTrigger msg;
	msg.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
	msg.event_type = EVENT_TRIGGER_EVENT_TYPE_ASSISTANT;
	msg.result = EVENT_TRIGGER_RESULT_FAILURE;
	residentEventPublisher.publish(msg);
	
	// Wait until a message has been received
	ros::Rate loop_rate(10);
	ROS_INFO("About to start spinning");
	while(receivedEventTriggers.size() == 0) {
		loop_rate.sleep();
		ros::spinOnce();
	}
	ROS_INFO("Done spinning");
	
	/*** ASSERTIONS BEGIN ***/
	
	// Check we received only one EventTrigger
	ASSERT_EQ(1, receivedEventTriggers.size());
	
	msg = receivedEventTriggers[0];
	
	// Check that the message has the correct information
	ASSERT_EQ(EVENT_TRIGGER_MSG_TYPE_REQUEST, msg.msg_type);
	ASSERT_EQ(EVENT_TRIGGER_EVENT_TYPE_ASSISTANT, msg.event_type);
	ASSERT_EQ(EVENT_TRIGGER_RESULT_FAILURE, msg.result);
}

/**
 * Callback for the 'event_trigger' topic messages.
 * Simply logs the received message into a list.
 */
void eventTriggerCallback(EventTrigger msg) {
	receivedEventTriggers.push_back(msg);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "TestScheduler");
	ros::NodeHandle nodeHandle;
	ros::Rate loop_rate(10);

	// Advertise and subscribe to topics
	residentEventPublisher = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("resident_event",1000, true);
	ros::Subscriber eventTriggerSubscriber = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, eventTriggerCallback);
	
	// Run tests to see if we received messages as expected
	testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
