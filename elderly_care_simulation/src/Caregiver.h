#ifndef CAREGIVER_H
#define CAREGIVER_H

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
 * Represents a friend (visitor) of the Resident. Moves to
 * the Resident to perform a simple action when it is requested.
 *
 * Author: Matthew Chiam
 */

;class Caregiver : public Robot {
	public:
		Caregiver();
		~Caregiver();

		int MY_TASK;
		bool performingTask;

		// Location State
		enum LocationState {
			AT_HOME, AT_RESIDENT, GOING_TO_RESIDENT, GOING_HOME, GOING_TO_SHOWER, AT_SHOWER
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
        StaticPoi homePoi = StaticPoi(CAREGIVER_HOME_X, CAREGIVER_HOME_Y, 0.0f);
        StaticPoi showerPoi = StaticPoi(SHOWER_X, SHOWER_Y, 0.0f);

		void goToResident(const std_msgs::Empty);
		void goToHome(const std_msgs::Empty);
		void eventTriggerReply();
		void eventTriggerCallback(elderly_care_simulation::EventTrigger msg);
		void performTask();	
		void goToShower(const std_msgs::Empty);
};

#endif
