#ifndef ASSISTANT_H
#define ASSISTANT_H

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


class Visitor : public Robot{
	public:
		Visitor();
		~Visitor();

		int MY_TASK;
		bool performingTask;

		// Location State
		enum LocationState {
			AT_HOME, AT_RESIDENT, GOING_TO_RESIDENT, GOING_HOME
		};

		LocationState currentLocationState;
		
		ros::Subscriber eventTriggerSub;
		ros::Subscriber pathToRobotSub;
		ros::Subscriber pathToHomeSub;
		ros::Subscriber locationInstructionsSub;
		ros::Publisher eventTriggerPub;
		ros::ServiceClient performTaskClient;
		
		void goToResident(const std_msgs::Empty);
		void goToHome(const std_msgs::Empty);
		void eventTriggerReply();
		void startRotating();
		void stopRotating();
		void eventTriggerCallback(elderly_care_simulation::EventTrigger msg);
		void performTask();		

	private:

};
#endif