#ifndef STATICPOI_H
#define STATICPOI_H

#include "Poi.h"
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"

class StaticPoi : public Poi {
    public:
        StaticPoi(float x, float y, float z) {
            this->location.x = x;
            this->location.y = y;
            this->location.z = z;
        }
        ~StaticPoi();

        virtual geometry_msgs::Point getLocation() {
            return location;
        }

    private:
        geometry_msgs::Point location;

};

#endif