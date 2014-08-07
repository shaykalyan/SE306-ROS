#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf2_bullet.h>


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
geometry_msgs::Pose desiredLocation;

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
    double q0 = msg.pose.pose.orientation.x;
    double q1 = msg.pose.pose.orientation.y;
    double q2 = msg.pose.pose.orientation.z;
    double q3 = msg.pose.pose.orientation.w;

    double yaw,pitch,roll;
    btMatrix3x3(msg.pose.pose.orientation).getEulerYPR(yaw,pitch,roll);
    
    
    currentAngle = atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3));
    ROS_INFO("Current angle: %f", currentAngle);
    ROS_INFO("Quaternion:    %f, %f, %f, %f", q0, q1, q2, q3);
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
    // Find the correct angle
    //geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation

    //directionVector.linear.x = desiredLocation.position.x - currentLocation.position.x;
    //directionVector.linear.y = desiredLocation.position.y - currentLocation.position.y;
    //directionVector.linear.z = desiredLocation.position.z - currentLocation.position.z;
    
    // Thank god we're only doing 2D stuff
    //double desiredAngle = atan2(directionVector.linear.y, directionVector.linear.x);
    
    //if (! doubleEquals(currentAngle, desiredAngle, 0.1)) {
    //    // Turn towards angle
    //    currentVelocity.linear.x = 0;
    //    currentVelocity.linear.y = 0;
    //    currentVelocity.linear.z = 0;
    //    currentVelocity.angular.z = currentAngle - 
    //} else {
    //    // Go forward
    //    currentVelocity.linear.x = 1;
    //    currentVelocity.linear.y = 0;
    //    currentVelocity.linear.z = 0;
    //    currentVelocity.angular.z = 0;
    //}
}

bool doubleEquals(double a, double b, double difference)
{
    return std::abs(a - b) < difference;
}

bool atDesiredLocation()
{  
    double toleratedDifference = 0.15;
    //logLocation("Current location: ", currentLocation);
    //logLocation("Desired location: ", desiredLocation);
    //return false;    
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

    currentAngle = 0.0;
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
            currentVelocity.angular.z = -1;
            stagePublisher.publish(currentVelocity);
        }
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
