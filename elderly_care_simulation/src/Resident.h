#ifndef RESIDENT_H
#define RESIDENT_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "EventTriggerConstants.h"
#include "PerformTaskConstants.h"
#include "elderly_care_simulation/PerformTask.h"
#include <queue>
#include <tf/tf.h>
#include "std_msgs/Empty.h"
#include <sstream>
#include "math.h"
#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"
#include "EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <unistd.h>

#include "Robot.h"
#include "Poi.h"

class Resident : public Robot, public Poi {

	public:
		Resident();
		~Resident();
		int currentTaskType;
		int HEALTHY_THRESHOLD;
		int happiness;
		int amusement;

		ros::Subscriber diceTriggerSub;
		ros::Publisher residentEventPub;
		ros::Subscriber locationInstructionsSub;
		ros::Subscriber pathOfResidentSub;

		void diceTriggerCallback(elderly_care_simulation::DiceRollTrigger msg);
		void taskCompleted(const std_msgs::Empty);
		int handleTask(int taskType);
		bool performTaskServiceHandler(elderly_care_simulation::PerformTask::Request &req,
				   elderly_care_simulation::PerformTask::Response &res);

        virtual geometry_msgs::Point getLocation() {
            return currentLocation.position;
        }

	private:
		
};

#endif
