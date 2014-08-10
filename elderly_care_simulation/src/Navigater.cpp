#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>


#include <sstream>
#include <math.h>
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
geometry_msgs::Point desiredLocation;

// Current Velocity
geometry_msgs::Twist currentVelocity;

// Current Angle
double currentAngle;

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
    double x = msg.pose.pose.orientation.x;
    double y = msg.pose.pose.orientation.y;
    double z = msg.pose.pose.orientation.z;
    double w = msg.pose.pose.orientation.w;
    
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
    currentAngle = yaw;
}

void updateDesiredLocationCallback(const geometry_msgs::Point location)
{   
    // Update desired location
    desiredLocation = location;
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

bool doubleEquals(double a, double b, double difference)
{
    return std::abs(a - b) < difference;
}

double normalizeAngle(double angle)
{
    while (angle < 0) {
        angle += 2 * M_PI;
    }
    while (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}

bool turnAnticlockwise(double currentAngle, double desiredAngle)
{   
    if (currentAngle < 0) {
        currentAngle = 2 * M_PI + currentAngle;
    }
    if (desiredAngle < 0) {
        desiredAngle = 2 * M_PI + desiredAngle;
    }
    desiredAngle = normalizeAngle(desiredAngle - currentAngle);
    return desiredAngle < M_PI;
    
}

void updateCurrentVelocity()
{
    // Find the correct angle
    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation

    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;
    
    // Thank god we're only doing 2D stuff
    double desiredAngle = atan2(directionVector.y, directionVector.x);

    ROS_INFO("DESIRED ANGLE: %f", desiredAngle);
    ROS_INFO("CURRENT ANGLE: %f", currentAngle);

    
    if (! doubleEquals(currentAngle, desiredAngle, 0.2)) {
        // Turn towards angle
        currentVelocity.linear.x = 0;
        currentVelocity.linear.y = 0;
        currentVelocity.linear.z = 0;
        
        if (turnAnticlockwise(currentAngle, desiredAngle)) {
            // Turn anti clockwise
            currentVelocity.angular.z = 1;
        } else {
            // Turn clockwise
            currentVelocity.angular.z = -1;
        }
    } else {
        // Go forward
        currentVelocity.linear.x = 1;
        currentVelocity.linear.y = 0;
        currentVelocity.linear.z = 0;
        currentVelocity.angular.z = 0;
    }
}

bool atDesiredLocation()
{  
    double toleratedDifference = 0.15;
    //logLocation("Current location: ", currentLocation);
    //logLocation("Desired location: ", desiredLocation);
    //return false;    
    return doubleEquals(currentLocation.position.x, desiredLocation.x, toleratedDifference) &&
           doubleEquals(currentLocation.position.y, desiredLocation.y, toleratedDifference) &&
           doubleEquals(currentLocation.position.z, desiredLocation.z, toleratedDifference);
      
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

    currentAngle = 0.0;
    
    // Set starting location
    desiredLocation.x = currentLocation.position.x;
    desiredLocation.y = currentLocation.position.y;
    desiredLocation.z = currentLocation.position.z;
    
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
