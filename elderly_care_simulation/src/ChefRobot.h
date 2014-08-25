#ifndef CHEF_H
#define CHEF_H

#include <ros/ros.h>

#include "geometry_msgs/Point.h"
#include "Poi.h"
#include "Robot.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

class ChefRobot : public Robot {
    public:
        Chef();
        ~Chef();
        void eventTriggered(const elderly_care_simulation::EventTrigger msg);
        int execute();

    private:
        StaticPoi base  = StaticPoi(12.0f, 0, 0);
        StaticPoi stove = StaticPoi(STOVE_X, STOVE_Y, 0.0f);

        // Location State
		enum LocationState {
			AT_BASE, GOING_TO_BASE, AT_STOVE, GOING_TO_STOVE
		};

		LocationState currentLocationState;

		void goToBase();
		void goToStove();

};

#endif