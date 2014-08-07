#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include <sstream>
#include "math.h"
#include <string>

// Publisher to stage
ros::Publisher stagePublisher;

// Subscriber to stage
ros::Subscriber stageSubscriber;

// Subscriber to navigation locations;
ros::Subscriber navigationSubscriber;

// Current location of the robot
geometry_msgs::Pose currentLocation;

// Desired location of the robot
geometry_msgs::Pose desiredLocation;

// Current Velocity
geometry_msgs::Twist currentVelocity;

void logLocation(std::string s, geometry_msgs::Pose p)
{
    // Log current location details
    double x, y, z;
    x = p.position.x;
    y = p.position.y;
    z = p.position.z;
    ROS_INFO("%s: %f, %f, %f", s.c_str(), x, y, z);
}

void stageOdometryCallback(const nav_msgs::Odometry msg)
{
    //Update Current Position
    currentLocation = msg.pose.pose;
    ROS_INFO("Quaternion angle: \nx: %f\ny: %f\nz: %f\nw: %f", msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
}

void updateDesiredLocationCallback(const geometry_msgs::Pose location)
{   
    // Update desired location
    desiredLocation = location;
    
    // Log current location details
    logLocation("Desired Location is now:", desiredLocation);
}

void normalize(geometry_msgs::Vector3 &vector)
{  
    ROS_INFO("Initial Vector: %f, %f, %f", vector.x, vector.y, vector.z);
    double length = vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
    length = sqrt(length);
    vector.x = 0.1 * round(10 * (vector.x / length));
    vector.y = 0.1 * round(10 * (vector.y / length));
    vector.z = 0.1 * round(10 * (vector.z / length));
    ROS_INFO("Normalized Vector: %f, %f, %f", vector.x, vector.y, vector.z);
    
}

void updateCurrentVelocity()
{
    // Calculate Direction to travel
    currentVelocity.linear.x = desiredLocation.position.x - currentLocation.position.x;
    currentVelocity.linear.y = desiredLocation.position.y - currentLocation.position.y;
    currentVelocity.linear.z = desiredLocation.position.z - currentLocation.position.z;
    
    // Normalizes direction
    normalize(currentVelocity.linear);
}

bool doubleEquals(double a, double b, double difference)
{
    return std::abs(a - b) < difference;
}

bool atDesiredLocation()
{  
    double toleratedDifference = 0.15;
    logLocation("Current location: ", currentLocation);
    logLocation("Desired location: ", desiredLocation);
    return doubleEquals(currentLocation.position.x, desiredLocation.position.x, toleratedDifference) &&
           doubleEquals(currentLocation.position.y, desiredLocation.position.y, toleratedDifference) &&
           doubleEquals(currentLocation.position.z, desiredLocation.position.z, toleratedDifference);
      
}

void initializeRobot(int argc, char **argv)
{
    ros::init(argc, argv, "Navigation_Robot");

    // Node handle
    ros::NodeHandle robotNodeHandle;
    
    // Will publish geometry_msgs::Twist messages to the cmd_vel topic
    stagePublisher = robotNodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    // Will listen to messages coming from stage
    stageSubscriber = robotNodeHandle.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 1000, stageOdometryCallback);

    // Subscribe to the navigate topic
    navigationSubscriber = robotNodeHandle.subscribe("navigation", 1000, updateDesiredLocationCallback);
}

int main(int argc, char **argv)
{
    initializeRobot(argc, argv);
    
    ros::Rate loopRate(10);
    while (ros::ok())
    {
        // Tell the robot which way to go
        if (! atDesiredLocation()) {
            updateCurrentVelocity();
            stagePublisher.publish(currentVelocity);
        }
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
