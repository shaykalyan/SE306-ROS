#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"

#include <sstream>
#include "math.h"
#include <cstdlib>

#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <queue>

class Robot{

	public:
		Robot();
		~Robot();
		double  currentAngle;

		geometry_msgs::Twist currentVelocity;
		geometry_msgs::Pose currentLocation;
		ros::ServiceClient pathFinderService;
		ros::Publisher robotNodeStagePub;
		ros::Subscriber stageOdoSub;
		std::queue<geometry_msgs::Point> locationQueue;

		void stage0domCallback(const nav_msgs::Odometry msg);
		void addPointsToQueue(const std::vector<geometry_msgs::Point> points);
		void updateDesiredLocationCallback(const geometry_msgs::Point location);
		void clearLocationQueue();
		bool doubleEquals(double a, double b, double difference);
		double normalizeAngle(double angle);
		bool turnAnticlockwise(double currentAngle, double desiredAngle);
		bool atDesiredLocation();
		void updateCurrentVelocity();
		

	private:

};

#endif