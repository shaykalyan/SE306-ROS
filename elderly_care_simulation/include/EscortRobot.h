#ifndef ESCORT_H
#define ESCORT_H

#include <ros/ros.h>

#include "elderly_care_simulation/EventTrigger.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "Robot.h"

class EscortRobot : public Robot {
    public:
        EscortRobot();
        EscortRobot(int escortEventType, geometry_msgs::Point escortBase, geometry_msgs::Point escortPoi);
        ~EscortRobot();
        void eventTriggered(const elderly_care_simulation::EventTrigger msg);
        int execute();
        void residentLocationCallback(nav_msgs::Odometry msg);

        ros::Publisher eventTriggerPub;
        ros::Subscriber eventTriggerSub;
        ros::Subscriber residentLocationSub;

        ros::ServiceClient performTaskClient;

    private:
        geometry_msgs::Point base;
        geometry_msgs::Point poi;
        geometry_msgs::Point residentLocation;

        int eventType;
        bool performingTask;

        // Location State
        enum LocationState {
            AT_BASE, GOING_TO_BASE, AT_POI, GOING_TO_POI
        };

        LocationState currentLocationState;

        void eventFinished();
        void goToBase();
        void goToPoi();
        void goToResident();
        void performTask();
        void notifySchedulerOfTaskCompletion();

};

#endif