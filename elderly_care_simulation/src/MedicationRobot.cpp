#include "ros/ros.h"
#include <unistd.h>
#include <sstream>

#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

#include <tf/tf.h>
#include <queue>

#include "EventTriggerUtility.h"
#include "PerformTaskConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include "elderly_care_simulation/PerformTask.h"
#include "elderly_care_simulation/FindPath.h"

#include "Robot.h"
#include "MedicationRobot.h"

#include "Poi.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

/**
 * Represents a medication providing robot for the Resident. Moves to
 * the Resident to perform a simple action when it is requested.
 *
 * Author: James Butler
 */

;MedicationRobot::MedicationRobot() {
    MY_TASK = EVENT_TRIGGER_EVENT_TYPE_MEDICATION;
    performingTask = false;
    currentLocationState = AT_HOME;
}

MedicationRobot::~MedicationRobot() {  
}

/**
 * Request robot to start moving towards the Resident
 */
void MedicationRobot::goToResident(const std_msgs::Empty) {
    ROS_INFO("MedicationRobot: Going to %f, %f", residentPoi.getLocation().x, residentPoi.getLocation().y);
    goToLocation(residentPoi.getLocation());
    currentLocationState = GOING_TO_RESIDENT;
}

/**
 * Request robot to move back to its home location
 */
void MedicationRobot::goToHome(const std_msgs::Empty) {
    goToLocation(homePoi.getLocation());
    currentLocationState = GOING_HOME;
}

/**
 * Publish completion report of this robot's task
 */
void MedicationRobot::eventTriggerReply() {
    // Create response message
    elderly_care_simulation::EventTrigger msg;
    msg.msg_type = EVENT_TRIGGER_MSG_TYPE_RESPONSE;
    msg.event_type = MY_TASK;
    msg.event_priority = EVENT_TRIGGER_PRIORITY_UNDEFINED;
    msg.event_weight = getEventWeight(msg.event_type);
    msg.result = EVENT_TRIGGER_RESULT_SUCCESS;

    eventTriggerPub.publish(msg);
    ROS_INFO("MedicationRobot: Reply Message Sent");
}

/**
 * Perform task when a request is received. This callback
 * method should be called when getting a request from
 * the Scheduler. 
 */
void MedicationRobot::eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
    if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_REQUEST) {

        if (msg.event_type == MY_TASK) {
            ROS_INFO("MedicationRobot: Event Recieved: [%s]", eventTypeToString(MY_TASK));
            
            performingTask = true;
            
            std_msgs::Empty emptyMessage;
            goToResident(emptyMessage);
            
        }
    }
}

/**
 * Perform a task on the resident by making a service call to them.
 */
void MedicationRobot::performTask() {
    
    // Generate the service call
    elderly_care_simulation::PerformTask performTaskSrv;
    performTaskSrv.request.taskType = MY_TASK;
    
    // Make the call using the client
    if (!performTaskClient.call(performTaskSrv)) {
        throw std::runtime_error("MedicationRobot: Service call to the initiate task with Resident failed");
    }
    
    switch (performTaskSrv.response.result) {
        case PERFORM_TASK_RESULT_ACCEPTED:
        {
            // Resident has accepted the task but isn't finished yet
            startSpinning(false);
            break;
        }
        case PERFORM_TASK_RESULT_FINISHED:
        {
            // Resident accepted the task and has had enough            
            performingTask = false;
            
            std_msgs::Empty emptyMessage;
            goToHome(emptyMessage);
            
            stopSpinning();
            eventTriggerReply();
            break;
        }
        case PERFORM_TASK_RESULT_BUSY:
        {
            // Resident is busy, do nothing
            break;
        }
    }
}
