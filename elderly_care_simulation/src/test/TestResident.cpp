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

    // ===========================================================================

    int companionshipResult = performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP);
    
    // Ensure that the resident accepts the moral support
    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, companionshipResult);

    // ===========================================================================

    // Make a EVENT_TRIGGER_EVENT_TYPE_VERY_ILL request
    int veryIllResult = performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_VERY_ILL);
    
    // Ensure that the resident accepts the very ill request
    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, veryIllResult);

    // ===========================================================================

    // Make another companionship call
    companionshipResult = performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP);

    // The response should tell us to finish
    ASSERT_EQ(PERFORM_TASK_RESULT_FINISHED, companionshipResult);
}

/**
 * If a visitor is providing a low priority task like companionship, a nurse should be 
 * able to interrupt with an ILL event type.
 */
 TEST_F(ResidentTest, interruptCompanionshipWithIllEvent) {
    int companionshipResult = performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP);
    
    // Ensure that the resident accepts the moral support
    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, companionshipResult);

    // ===========================================================================

    // Make a EVENT_TRIGGER_EVENT_TYPE_VERY_ILL request
    int illResult = performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_ILL);
    
    // Ensure that the resident accepts the very ill request
    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, illResult);

    // ===========================================================================

    // Make another companionship call
    companionshipResult = performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP);

    // The response should tell us to finish
    ASSERT_EQ(PERFORM_TASK_RESULT_FINISHED, companionshipResult);
}

/**
 * If a visitor is providing a low priority task like companionship, another helper robot 
 * requesting to perform another low priority task should be told to wait.
 */
 TEST_F(ResidentTest, interruptCompanionshipWithConversation) {
    int companionshipResult = performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP);
    
    // Ensure that the resident accepts the moral support
    ASSERT_EQ(PERFORM_TASK_RESULT_ACCEPTED, companionshipResult);

    // ===========================================================================

    // Make a EVENT_TRIGGER_EVENT_TYPE_VERY_ILL request
    int conversationResult = performTaskOnResident(EVENT_TRIGGER_EVENT_TYPE_CONVERSATION);
    
    // Ensure that the resident accepts the very ill request
    ASSERT_EQ(PERFORM_TASK_RESULT_BUSY, conversationResult);
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