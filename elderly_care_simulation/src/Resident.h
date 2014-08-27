#ifndef RESIDENT_H
#define RESIDENT_H

#include "ros/ros.h"
#include <sstream>
#include <unistd.h>

#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Empty.h"
#include <geometry_msgs/Point.h>

#include <queue>
#include <tf/tf.h>

#include "PerformTaskConstants.h"
#include "elderly_care_simulation/PerformTask.h"
#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"
#include "EventTriggerUtility.h"
#include "elderly_care_simulation/EventTrigger.h"

#include "Robot.h"
#include "Poi.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

class Resident : public Robot, public Poi {

	public:
		Resident();
		~Resident();

		int currentTaskType;

		std::map<int, int> taskProgress;

		// Movement State
		enum MovementState {
			STATIONARY, MOVING
		};

		enum MovementTarget {
			NONE, KITCHEN, BEDROOM, HALLWAY, BED
		};

		MovementState currentMovementState;
		MovementTarget currentMovementTarget;

		ros::Publisher externalEventPub;
		ros::Publisher eventTriggerPub;
		ros::Subscriber diceTriggerSub;
		ros::Subscriber eventTriggerSub;
		ros::Subscriber locationInstructionsSub;
		ros::Subscriber pathOfResidentSub;

		void diceTriggerCallback(elderly_care_simulation::DiceRollTrigger msg);
		void taskCompleted(const std_msgs::Empty);
		void clearAllTasks();
		int handleTask(int taskType);
		bool performTaskServiceHandler(elderly_care_simulation::PerformTask::Request &req,
				   elderly_care_simulation::PerformTask::Response &res);
		void eventTriggerReply(int eventType);
		void eventTriggerCallback(elderly_care_simulation::EventTrigger msg);

        StaticPoi kitchenPoi = StaticPoi(KITCHEN_X, KITCHEN_Y, 0.0f);
        StaticPoi hallwayPoi = StaticPoi(HALLWAY_X, HALLWAY_Y, 0.0f);
        StaticPoi bedroomPoi = StaticPoi(BEDROOM_X, BEDROOM_Y, 0.0f);
        StaticPoi bedPoi = StaticPoi(BED_X, BED_Y, 0.0f);

        virtual geometry_msgs::Point getLocation() {
            return currentLocation.position;
        }

	private:
		void resetTaskProgress(int taskType);
		bool shouldRespondGoAway(int requestedTaskType);
		bool shouldOverrideCurrentTask(int requestedTaskType);
		
};

#endif
