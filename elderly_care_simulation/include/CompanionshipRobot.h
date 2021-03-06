#ifndef COMPANIONSHIPROBOT_H
#define COMPANIONSHIPROBOT_H

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
 * This robot represents a robot assistant which is responsible
 * for providing companionship. Companionship is modelled via
 * the robot performing a simple action, such as spinning when
 * the robot has approached the resident. 
 *
 * Providing companionship reflects the situation where
 * the resident communites to external beings via VoIP software
 * such as Skype.
 *
 * Author: Akshay Kalyan
 */

;class CompanionshipRobot : public Robot {
	public:
		CompanionshipRobot();
		~CompanionshipRobot();

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
		
        StaticPoi residentPoi = StaticPoi(0.0f, 0.0f, 0.0f);
        StaticPoi homePoi = StaticPoi(COMPANIONSHIP_HOME_X, COMPANIONSHIP_HOME_Y, 0.0f);

		void goToResident(const std_msgs::Empty);
		void goToHome(const std_msgs::Empty);
		void eventTriggerReply();
		void eventTriggerCallback(elderly_care_simulation::EventTrigger msg);
		void performTask();		
};

#endif