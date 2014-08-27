#include "ros/ros.h"

#include <sstream>
#include <math.h>
#include <cstdlib>

#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <queue>
#include "Robot.h"
#include <vector>
#include "elderly_care_simulation/FindPath.h"

Robot::Robot() {
    spin = NOT_SPINNING;
    outsideBounds = false;
}

Robot::~Robot() {
}

/**
 * Starts the robot spinning clockwise or anticlockwise.
 */ 
void Robot::startSpinning(bool clockwise) {

    if (clockwise) {
        spin = CLOCKWISE;
    } else {
        spin = ANTI_CLOCKWISE;
    }
}

/**
 * Stops the robot spinning
 */ 
void Robot::stopSpinning() {

    spin = NOT_SPINNING;
}

/**
 * Checks whether the robot has been *manually* moved.
 * If the robot has, then the robot will find a new path 
 * to the desired location from the current location.
 */
void Robot::checkForMovement() {

    if (!locationQueue.empty()) {
        geometry_msgs::Point currentDestination = locationQueue.front();

        geometry_msgs::Point d;
        d.x = currentDestination.x - currentLocation.position.x;
        d.y = currentDestination.y - currentLocation.position.y;

        float distance = sqrt(d.x * d.x + d.y * d.y);
        if (distance > 2) {
            goToLocation(finalDestination);
        }
    }
}

/**
 * Checks to see whether the robot is within tolerance of the given POI.
 */ 
bool Robot::atPointOfInterest(geometry_msgs::Point p, double tolerance) {

    geometry_msgs::Point d;
    d.x = p.x - currentLocation.position.x;
    d.y = p.y - currentLocation.position.y;

    float distance = sqrt(d.x * d.x + d.y * d.y);
    return distance <= tolerance;
}

/**
 * Updates current position of the robot from Stage
 */
void Robot::stage0domCallback(const nav_msgs::Odometry msg) {
	
    currentLocation = msg.pose.pose;
    double x = currentLocation.orientation.x;
    double y = currentLocation.orientation.y;
    double z = currentLocation.orientation.z;
    double w = currentLocation.orientation.w;
  	double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
    currentAngle = yaw;

    checkForMovement();
}

/**
 * Add all points in points to the location queue.
 */ 
void Robot::addPointsToQueue(const std::vector<geometry_msgs::Point> points) {

    for (uint i = 0; i < points.size(); ++i) {
        locationQueue.push(points[i]);
    }
}

/**
 * Clears the location queue.
 */ 
void Robot::clearLocationQueue()
{
    while (! locationQueue.empty()) {
        locationQueue.pop();
    }
}

/**
 * Adds location's points to the queue to traverse 
 * If the robot is outside the bounds of the map, the robot will head towards 0,0 until it is inside the map.
 */
void Robot::goToLocation(const geometry_msgs::Point location, bool closeEnough /*= false*/) { 

    clearLocationQueue();
    elderly_care_simulation::FindPath srv;
    srv.request.from_point = currentLocation.position;
    srv.request.to_point = location;
    if (pathFinderService.call(srv)) {
        std::vector<geometry_msgs::Point> points = srv.response.path;
        if (closeEnough) {
            points.pop_back();
        }
        outsideBounds = false;
        addPointsToQueue(points);
    } else {
        ROS_INFO("Call failed");
        std::vector<geometry_msgs::Point> points;
        geometry_msgs::Point origin;
        points.push_back(origin);
        addPointsToQueue(points);
        outsideBounds = true;
    }

    finalDestination.x = location.x;
    finalDestination.y = location.y;
    previousCloseEnough = closeEnough;
}

/**
 * Check if a == b with some level of leniency
 */
bool Robot::doubleEquals(double a, double b, double difference) {

    return std::abs(a - b) < difference;
}

/**
 * Normalises given angle
 */
double Robot::normalizeAngle(double angle) {

    while (angle < 0) {
        angle += 2 * M_PI;
    }
    while (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}


/**
 * Turns robot anticlockwise by a given angle
 */
bool Robot::turnAnticlockwise(double currentAngle, double desiredAngle) {  
 
    if (currentAngle < 0) {
        currentAngle = 2 * M_PI + currentAngle;
    }
    if (desiredAngle < 0) {
        desiredAngle = 2 * M_PI + desiredAngle;
    }
    desiredAngle = normalizeAngle(desiredAngle - currentAngle);
    return desiredAngle < M_PI;
    
}

/**
 * Returns true if the robot is close enough to the final
 * target's location and false otherwise. 
 */
bool Robot::atDesiredLocation() {  

    if (locationQueue.empty()) {
        return true;
    } else {
        double toleratedDifference = 0.15;
        geometry_msgs::Point desiredLocation = locationQueue.front();

        if (doubleEquals(currentLocation.position.x, desiredLocation.x, toleratedDifference) &&
            doubleEquals(currentLocation.position.y, desiredLocation.y, toleratedDifference)) {
            locationQueue.pop();
            return atDesiredLocation();
        }
    }
    return false;
      
}

/**
 * Calculates the difference in radians between the two angles.
 * Angles should be between -pi and pi.
 */
double Robot::differenceInAngle(double current, double desired) {

    current = current > 0 ? current : current + 2 * M_PI;
    desired = desired > 0 ? desired : desired + 2 * M_PI;

    double difference = std::abs(current - desired);
    difference = difference <= M_PI ? difference : (2 * M_PI) - difference;
    return difference;

}

/**
 * Updates the current velocity to head towards the finalDestination
 * via the points in the locationQueue.
 */
void Robot::updateCurrentVelocityToDesiredLocation() {

    if (locationQueue.empty()) {
        return;
    }

    // Find the correct angle
    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation

    geometry_msgs::Point desiredLocation = locationQueue.front();

    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    
    double desiredAngle = atan2(directionVector.y, directionVector.x);

    double angle = differenceInAngle(currentAngle, desiredAngle);
    if (angle > 0.05) {
        // Turn towards angle

        // If the amount we need to turn is more than 45 degrees
           // Turn without going forward
        // Otherwise
           // Turn while going forward
        double angularVelocity = 0;
        double linearVelocity = 0;

        ROS_INFO("Angle: %f", angle);
        if (angle > M_PI) {
            angularVelocity = M_PI * 2;

        } else if (angle > (M_PI / 2)) {
            angularVelocity = M_PI;

        } else if (angle > (M_PI / 8)) {
            angularVelocity = M_PI / 4;

        } else {
            angularVelocity = M_PI / 8;
        }

        if (! turnAnticlockwise(currentAngle, desiredAngle)) {
            // Turn clockwise
            angularVelocity *= -1;
        }

        ROS_INFO("Velocty: %f", angularVelocity);

        currentVelocity.linear.x = linearVelocity;
        currentVelocity.angular.z = angularVelocity;
    } else {
        // Go forward
        currentVelocity.linear.x = 10;
        currentVelocity.angular.z = 0;
    }
}

/**
 * If at the desired location, stop the robot's movement.
 * Otherwise, turns towards the next location and move towards it.
 */
void Robot::updateCurrentVelocity() {
    if (outsideBounds) {
        goToLocation(finalDestination, previousCloseEnough);
    }

    if (spin == CLOCKWISE) {
        // Spin clockwise
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = M_PI * 2;
    } else if (spin == ANTI_CLOCKWISE) {
        // Spin anti_clockwise
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = - M_PI * 2;
    } else if (atDesiredLocation()) {
        // Stop robot
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = 0;
    } else {
        updateCurrentVelocityToDesiredLocation();
    }
    robotNodeStagePub.publish(currentVelocity);
}
