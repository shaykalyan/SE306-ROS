#ifndef CHEFROBOT_H
#define CHEFROBOT_H

#include <ros/ros.h>

#include "elderly_care_simulation/EventTrigger.h"
#include "geometry_msgs/Point.h"
#include "Poi.h"
#include "Robot.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

;class ChefRobot : public Robot {
    public:
        ChefRobot();
        ~ChefRobot();
        void eventTriggered(const elderly_care_simulation::EventTrigger msg);
        int execute();

        ros::Publisher eventTriggerPub;
        ros::Subscriber eventTriggerSub;

    private:
        StaticPoi base = StaticPoi(1.0f, 0.0f, 0.0f);
        StaticPoi stove = StaticPoi(STOVE_X, STOVE_Y, 0.0f);

        // Location State
        enum LocationState {
            AT_BASE, GOING_TO_BASE, AT_STOVE, GOING_TO_STOVE
        };

        LocationState currentLocationState;

        void eventFinished();
        void goToBase();
        void goToStove();

};

#endif