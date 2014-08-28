#ifndef POI_H
#define POI_H

#include "ros/ros.h"
#include <geometry_msgs/Point.h>

class Poi {
    
    public:
        Poi();
        ~Poi();

        virtual geometry_msgs::Point getLocation()=0;
};

#endif
