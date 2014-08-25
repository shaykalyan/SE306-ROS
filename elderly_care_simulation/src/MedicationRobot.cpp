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
}

MedicationRobot::~MedicationRobot() {  
}

/**
 * Request robot to start moving towards the Resident
 */
void MedicationRobot::goToResident(const std_msgs::Empty) {
}

/**
 * Request robot to move back to its home location
 */
void MedicationRobot::goToHome(const std_msgs::Empty) {
}

/**
 * Publish completion report of this robot's task
 */
void MedicationRobot::eventTriggerReply() {
}

/**
 * Perform task when a request is received. This callback
 * method should be called when getting a request from
 * the Scheduler. 
 */
void MedicationRobot::eventTriggerCallback(elderly_care_simulation::EventTrigger msg) {
}

/**
 * Perform a task on the resident by making a service call to them.
 */
void MedicationRobot::performTask() {
}
