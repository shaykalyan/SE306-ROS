#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>
#include "EventTriggerUtility.h"
#include "PerformTaskConstants.h"
#include "elderly_care_simulation/EventTrigger.h" 
using namespace elderly_care_simulation;
#include "elderly_care_simulation/PerformTask.h" 
using namespace elderly_care_simulation;
#include "EventNode.h"
#include <unistd.h> // sleep

#include "gtest/gtest.h"


// Publishers
//ros::Publisher residentEventPublisher;

// Subscribers
//ros::Subscriber eventTriggerSubscriber;

// Service Clients
ros::ServiceClient performTaskClient;

int performTaskOnResident(int taskType) {
    PerformTask taskRequest;
    taskRequest.request.taskType = taskType;
    
    // Make the call using the client
    if (!performTaskClient.call(taskRequest)) {
        throw std::runtime_error("Service call to the initiate task with Resident failed");
    }
    
    // Ensure that the resident accepts the moral support
    return taskRequest.response.result;
}


/**
 * The fixture for testing the Resident.
 * 
 * Most of these methods could be removed, but they have been left as
 * an example.
 */
class ResidentTest : public ::testing::Test {
    protected:
        // You can remove any or all of the following functions if its body
        // is empty.

        ResidentTest() {
            // You can put any initialisation here.
        }

        virtual ~ResidentTest() {
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

            // Clear the resident's tasks
            performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_UNDEFINED);
        }
};

/** 
 * If a visitor is providing a low priority task like companionship, a doctor should be 
 * able to interrupt with a VERY_ILL event type.
 */
TEST_F(ResidentTest, interruptCompanionshipWithVeryIllEvent) {

    // Sleep to allow the Resident to start
    ros::Rate loop_rate(2);
    loop_rate.sleep();
    
    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP));
    
    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_VERY_ILL));

    ASSERT_EQ(PERFORM_TASK_RESULT_FINISHED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP));

}

/**
 * If a visitor is providing a low priority task like companionship, a nurse should be 
 * able to interrupt with an ILL event type.
 */
 TEST_F(ResidentTest, interruptCompanionshipWithIllEvent) {

    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP));

    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_ILL));

    ASSERT_EQ(PERFORM_TASK_RESULT_FINISHED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP));

}

/**
 * If a visitor is providing a low priority task like companionship, another helper robot 
 * requesting to perform another low priority task should be told to wait.
 */
 TEST_F(ResidentTest, interruptCompanionshipWithConversationShouldWait) {
    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP));
    
    ASSERT_EQ(PERFORM_TASK_RESULT_BUSY, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_CONVERSATION));
}

/**
 * If a nurse is providing the ILL task, attempts to provide the CONVERSATION task should
 * result in a FINISHED response (i.e. told to go away.)
 */
 TEST_F(ResidentTest, interruptIllTaskWithConversationShouldGoAway) {    

    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_ILL));

    ASSERT_EQ(PERFORM_TASK_RESULT_FINISHED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_CONVERSATION));

}

/**
 * If a nurse is providing the ILL task, attempts to provide the VERY_ILL task should
 * be accepted and the ILL provider should be told to finish.
 */
 TEST_F(ResidentTest, interruptIllTaskWithVeryIllTask) {    

    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_ILL));

    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_VERY_ILL));

    ASSERT_EQ(PERFORM_TASK_RESULT_FINISHED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_ILL));

}

/**
 * If a doctor is providing the VERY ILL task, attempts to provide the CONVERSATION task should
 * result in a FINISHED response (i.e. told to go away.)
 */
 TEST_F(ResidentTest, interruptVeryIllTaskWithConversationShouldGoAway) {    

    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_VERY_ILL));

    ASSERT_EQ(PERFORM_TASK_RESULT_FINISHED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_CONVERSATION));

}

/**
 * If a doctor is providing the VERY ILL task, attempts to provide the ILL task should
 * result in a FINISHED response (i.e. told to go away.)
 */
 TEST_F(ResidentTest, interruptVeryIllTaskWithIllTaskShouldGoAway) {    

    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_VERY_ILL));

    ASSERT_EQ(PERFORM_TASK_RESULT_FINISHED, performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_ILL));

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "TestResident");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);

    // Advertise and subscribe to topics
    //residentEventPublisher = nodeHandle.advertise<elderly_care_simulation::EventTrigger>("resident_event",1000, true);
    //ros::Subscriber eventTriggerSubscriber = nodeHandle.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, eventTriggerCallback);
    
    performTaskClient = nodeHandle.serviceClient<PerformTask>("perform_task");

    // Run tests to see if we received messages as expected
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}