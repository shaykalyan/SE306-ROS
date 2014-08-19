#include "ros/ros.h"
#include "elderly_care_simulation/FindPath.h"
#include "math.h"

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Point.h>

// The map of this 
nav_msgs::OccupancyGrid occupancyGrid;

// Width and Height of the map in pixels
int width;
int height;

// Bottom Left position
float bottom;
float top;
float left;
float right;

// The number of pixels per meter in stage
float resolution;



/**
 * Helper function to return the int value 
 * at the x, y position in occupancyGrid.data.
 */
int occupancyAtPoint(int x, int y)
{
    return occupancyGrid.data[y * width + x];
}

/**
 * Returns a vector of geometry_msgs::Point objects that indicate the best path to take
 * from point 'from' to point 'to'.
 */
bool findPath(elderly_care_simulation::FindPath::Request  &req,
               elderly_care_simulation::FindPath::Response &res)
{
    // Find path
    ROS_WARN("Finding the path not implemented yet");
    return true;
}

/**
 * TODO: Remove
 */ 
void is_vacant(geometry_msgs::Point point)
{

    if (point.x < left || point.x > right ||
        point.y < bottom || point.y > top) {
        ROS_INFO("OUT OF BOUNDS at position %f, %f", point.x, point.y);
        return;
    }

    int x = round((resolution / (right - left)) * (point.x - left)   * width  / resolution);
    int y = round((resolution / (top - bottom)) * (point.y - bottom) * height / resolution);

    ROS_INFO("X: %d", x);
    ROS_INFO("Y: %d", y);

    if (occupancyAtPoint(x, y) > 0) {
        ROS_INFO("Wall at position %f, %f", point.x, point.y);
    } else {
        ROS_INFO("Empty at position %f, %f", point.x, point.y);
    }
}

void printFirst()
{
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
             if (occupancyGrid.data[y * width + x] > 0) {
                ROS_INFO("X: %d", x);
                ROS_INFO("Y: %d", y);
                ROS_INFO("Value: %d", occupancyGrid.data[y * width + x]);
                return;
             } 
        }
    }
}

void initializeHelperVariables()
{
    width      = occupancyGrid.info.width;
    height     = occupancyGrid.info.height;
    bottom     = occupancyGrid.info.origin.position.y;
    left       = occupancyGrid.info.origin.position.x;
    resolution = occupancyGrid.info.resolution;

    top        = bottom + (height / resolution);
    right      = left + (width / resolution);

    ROS_INFO("Width: %d", width);
    ROS_INFO("Height: %d", height);
    ROS_INFO("Bottom: %f", bottom);
    ROS_INFO("Top: %f", top);
    ROS_INFO("Left: %f", left);
    ROS_INFO("Right: %f", right);
    ROS_INFO("Resolution: %f", resolution);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Path_Server");

    // Node handle
    ros::NodeHandle pathNodeHandle;
    
    // Map server that offers a get_map service
    ros::ServiceClient mapServer = pathNodeHandle.serviceClient<nav_msgs::GetMap>("static_map");

    // TODO: Remove
    ros::Subscriber pathToRobotSub = pathNodeHandle.subscribe<geometry_msgs::Point>("is_vacant", 1000, is_vacant);

    nav_msgs::GetMap getMapService;
    if (mapServer.call(getMapService)) {
        occupancyGrid = getMapService.response.map;
        initializeHelperVariables();

        printFirst(); // TODO: Remove
    } else {
        ROS_WARN("Could not retrieve map from mapserver");
    }

    // Advertise that this Path Server offers the find_path service
    ros::ServiceServer pathService = pathNodeHandle.advertiseService("find_path", findPath);
    
    ros::spin();
    
    return 0;
}