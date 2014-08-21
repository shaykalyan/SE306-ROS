#include "Poi.h"
#include "StaticPoi.h"
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"

StaticPoi::StaticPoi(float x, float y, float z) {
    this->location.x = x;
    this->location.y = y;
    this->location.z = z;
}

StaticPoi::~StaticPoi() {
}
