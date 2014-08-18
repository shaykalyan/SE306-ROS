#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <queue>
#include <sstream>
#include "math.h"
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "math.h"
#include "Robot.h"


Robot::Robot(){

}
Robot::~Robot(){

}

void Robot::stage0domCallback(const nav_msgs::Odometry msg) {
	
	 //Update Current Position
    currentLocation = msg.pose.pose;
    double x = currentLocation.orientation.x;
    double y = currentLocation.orientation.y;
    double z = currentLocation.orientation.z;
    double w = currentLocation.orientation.w;
  	double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
    currentAngle = yaw;

}

void Robot::updateDesiredLocationCallback(const geometry_msgs::Point location)
{   
    // Add location to the locationQueue queue
    locationQueue.push(location);
}

bool Robot::doubleEquals(double a, double b, double difference)
{
    return std::abs(a - b) < difference;
}

double Robot::normalizeAngle(double angle)
{
    while (angle < 0) {
        angle += 2 * M_PI;
    }
    while (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}

bool Robot::turnAnticlockwise(double currentAngle, double desiredAngle)
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

bool Robot::atDesiredLocation()
{  
    if (locationQueue.empty()) {
        return true;
    } else {
        double toleratedDifference = 0.05;
        geometry_msgs::Point desiredLocation = locationQueue.front();

        if( doubleEquals(currentLocation.position.x, desiredLocation.x, toleratedDifference) &&
            doubleEquals(currentLocation.position.y, desiredLocation.y, toleratedDifference)) {
            locationQueue.pop();
            return atDesiredLocation();
        }
    }
    return false;
      
}

void Robot::updateCurrentVelocity()
{
    if (atDesiredLocation()) {
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = 0;
        return;
    }
    // Find the correct angle
    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation

    geometry_msgs::Point desiredLocation = locationQueue.front();

    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;
    
    // Thank god we're only doing 2D stuff
    double desiredAngle = atan2(directionVector.y, directionVector.x);

    if (! doubleEquals(currentAngle, desiredAngle, 0.1)) {
        // Turn towards angle
        currentVelocity.linear.x = 0;
        
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
        currentVelocity.angular.z = 0;
    }
}
