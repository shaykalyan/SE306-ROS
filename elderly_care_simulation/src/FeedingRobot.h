#ifndef FEEDER_H
#define FEEDER_H

#include <ros/ros.h>

#include "elderly_care_simulation/EventTrigger.h"
#include "geometry_msgs/Point.h"
#include "Poi.h"
#include "Robot.h"
#include "StaticPoi.h"
#include "StaticPoiConstants.h"

class FeedingRobot : public Robot {
    public:
        FeedingRobot();
        ~FeedingRobot();
        void eventTriggered(const elderly_care_simulation::EventTrigger msg);
        int execute();

        ros::Publisher eventTriggerPub;
        ros::Subscriber eventTriggerSub;

    private:
        StaticPoi base = StaticPoi(1.0f, 0.0f, 0.0f);
        StaticPoi table = StaticPoi(ADJACENT_TABLE_X, ADJACENT_TABLE_Y, 0.0f);

        // Location State
        enum LocationState {
            AT_BASE, GOING_TO_BASE, AT_TABLE, GOING_TO_TABLE
        };

        LocationState currentLocationState;

        void eventFinished();
        void goToBase();
        void goToTable();

};

#endif