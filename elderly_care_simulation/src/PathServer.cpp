#include "ros/ros.h"
#include "elderly_care_simulation/FindPath.h"
#include "math.h"

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Point.h>

// The map of this 
nav_msgs::OccupancyGrid occupancyGrid;

/**
 * Returns a vector of geometry_msgs::Point objects that indicate the best path to take
 * from point 'from' to point 'to'.
 * 
 */
bool findPath(elderly_care_simulation::FindPath::Request  &req,
               elderly_care_simulation::FindPath::Response &res)
{
    // Find path
    ROS_WARN("Finding the path not implemented yet");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Path_Server");

    // Node handle
    ros::NodeHandle pathNodeHandle;
    
    // Map server that offers a get_map service
    ros::ServiceClient mapServer = pathNodeHandle.serviceClient<nav_msgs::GetMap>("static_map");

    nav_msgs::GetMap getMapService;
    if (mapServer.call(getMapService)) {
        occupancyGrid = getMapService.response.map;
        ROS_INFO("Width: %d\nHeight: %d", occupancyGrid.info.width, occupancyGrid.info.height);
        ROS_INFO("Origin: %f, %f", occupancyGrid.info.origin.position.x, occupancyGrid.info.origin.position.y);
    } else {
        ROS_WARN("Could not retrieve map from mapserver");
    }

    // Advertise that this Path Server offers the find_path service
    ros::ServiceServer pathService = pathNodeHandle.advertiseService("find_path", findPath);
    
    ros::spin();
    
    return 0;
}