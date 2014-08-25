#ifndef NURSE_H
#define NURSE_H

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

#include "Robot.h"

#include "Poi.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

/**
 * This robot represents a nurse robot which is responsible
 * for providing necessary attention to the resident when he/she 
 * is ill. Medical treatment is modelled via
 * the robot performing a simple action, such as spinning when
 * the robot has approached the resident. 
 *
 * Providing medical attention reflects the situation where
 * the resident feels sick or ill, and requires external 
 * healthcare service and relavent expertise from the hospital.
 *
 * Author: Bert Huang
 */

;class Nurse : public Robot {
	public:
		Nurse();
		~Nurse();

		int MY_TASK;
		bool performingTask;

		// Location State
		enum LocationState {
			AT_HOME, AT_RESIDENT, GOING_TO_RESIDENT, GOING_HOME
		};

		LocationState currentLocationState;
		
        ros::Subscriber residentStageSub;
		ros::Subscriber eventTriggerSub;
		ros::Subscriber pathToRobotSub;
		ros::Subscriber pathToHomeSub;
		ros::Subscriber locationInstructionsSub;
		ros::Publisher eventTriggerPub;
		ros::ServiceClient performTaskClient;
		
        StaticPoi residentPoi = StaticPoi(1.0f, 0.0f, 0.0f);
        StaticPoi homePoi = StaticPoi(NURSE_HOME_X, NURSE_HOME_Y, 0.0f);

		void goToResident(const std_msgs::Empty);
		void goToHome(const std_msgs::Empty);
		void eventTriggerReply();
		void eventTriggerCallback(elderly_care_simulation::EventTrigger msg);
		void performTask();		
};

#endif