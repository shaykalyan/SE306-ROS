#include "ros/ros.h"
#include "elderly_care_simulation/FindPath.h"
#include "math.h"

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Point.h>

// The map to find paths on
nav_msgs::OccupancyGrid occupancyGrid;

// Width and Height of the map in pixels
int pixelWidth;
int pixelHeight;

// Bottom Left position
float bottom;
float top;
float left;
float right;

// Width height
float width;
float height;

// The number of pixels per meter in stage
float resolution;

std::vector<std::vector<int>> path;

/**
 * Helper function to return the int value 
 * at the x, y position in occupancyGrid.data.
 */
int occupancyAtPoint(int x, int y)
{
    return occupancyGrid.data[y * width + x] > 0 ? 0 : 1;
}

/**
 * Converts a x location on the map to the x index in the occupancyGrid
 */
int getXLocation(float x)
{
    return int(floor(x + 10));
}

/**
 * Converts a y location on the map to the y index in the occupancyGrid
 */
int getYLocation(float y)
{
    return int(floor(y + 10));
}

struct xyPoint
{
    int x;
    int y;
    bool visited;
    int distance;
    std::vector<xyPoint> path;

    bool operator<(const xyPoint& a, const xyPoint& b) {
        return a.distance < b.distance;
    }
};

std::set<xyPoint> initializePoints()
{
    std::set<xyPoint> points;
    for (int i = 0; i < width; i++) {
        for int j = 0; j < height; j++) {
            xyPoint p = {}
            points.insert()
        }
    }
}


void Dijkstra(int s)
{
    std::set<xyPoint> visited;
    std::set<xyPoint> notVisited = initializePoints();

    path = std::vector<std::vector<int>>(height, width);
    std::set<xyPoint> Q;

    int[][] 
    D[s] = 0;
    Q.insert(ii(0,s));
 
    while(!Q.empty())
    {
        xyPoint top = *Q.begin();
        Q.erase(Q.begin());
        int x = top.x;
        int y = top.y;
        int d = top.distance;
 
        for (vii::const_iterator it = G[v].begin(); it != G[v].end(); it++)
        {
            int v2 = it->first;
            int cost = it->second;
            if (D[v2] > D[v] + cost)
            {
                if (D[v2] != 1000000000)
                {
                    Q.erase(Q.find(ii(D[v2], v2)));
                }
                D[v2] = D[v] + cost;
                Q.insert(ii(D[v2], v2));
            }
        }
    }
}

/**
 * Returns a vector of geometry_msgs::Point objects that indicate the best path to take
 * from point 'from' to point 'to'.
 *
 * Code for this algorithm has been adapted from:
 * http://www.algolist.com/code/cpp/Dijkstra%27s_algorithm
 */
bool findPath(elderly_care_simulation::FindPath::Request& req,
              elderly_care_simulation::FindPath::Response& res)
{
    // Find path
    ROS_WARN("Finding the path not implemented yet");

    geometry_msgs::Point from = req.from;
    int startingX = getXLocation(from.x);
    int startingY = getYLocation(from.y);

    geometry_msgs::Point to = req.from;
    int finishingX = getXLocation(to.x);
    int finishingY = getYLocation(to.y);

    std::vector<geometry_msgs::Point> points;

    std::cout << "Start: " << startingX << ", " << startingY;
    std::cout << "Finish: " << finishingX << ", " << finishingY;

    return true;
}

bool is_vacant(geometry_msgs::Point point)
{

    int x = getXLocation(point.x);
    int y = getYLocation(point.y);

    //ROS_INFO("X: %d", x);
    //ROS_INFO("Y: %d", y);

    if (x < 0 || x >= width ||
        y < 0 || y >= height) {
        //ROS_INFO("OUT OF BOUNDS at position %f, %f", point.x, point.y);
        //return;
        return false;
    }

    if (occupancyAtPoint(x, y) > 0) {
        //ROS_INFO("Wall at position %f, %f", point.x, point.y);
        return true;
    } else {
        //ROS_INFO("Empty at position %f, %f", point.x, point.y);
        return false;
    }
    return true;
}

void initializeHelperVariables()
{
    pixelWidth  = occupancyGrid.info.width;
    pixelHeight = occupancyGrid.info.height;
    resolution  = occupancyGrid.info.resolution;

    bottom      = occupancyGrid.info.origin.position.y;
    left        = occupancyGrid.info.origin.position.x;
    top         = bottom + (pixelHeight / resolution);
    right       = left   + (pixelWidth  / resolution);

    width       = (right - left) * resolution;
    height      = (top - bottom) * resolution;

    ROS_INFO("Pixel width:  %d", pixelWidth);
    ROS_INFO("Pixel height: %d", pixelHeight);
    ROS_INFO("Resolution:   %f", resolution);

    ROS_INFO("Bottom:       %f", bottom);
    ROS_INFO("Left:         %f", left);
    ROS_INFO("Top:          %f", top);
    ROS_INFO("Right:        %f", right);
    
    ROS_INFO("Width:        %f", width);
    ROS_INFO("Height:       %f", height);
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
        initializeHelperVariables();
    } else {
        ROS_WARN("Could not retrieve map from mapserver");
    }

    // Advertise that this Path Server offers the find_path service
    ros::ServiceServer pathService = pathNodeHandle.advertiseService("find_path", findPath);
    
    ros::spin();
    
    return 0;
}