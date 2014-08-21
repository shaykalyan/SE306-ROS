#ifndef ROBOT_H
#define ROBOT_H

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
#include <math.h>


class Robot{

	public:
		Robot();
		~Robot();
		double  currentAngle;

		geometry_msgs::Twist currentVelocity;
		geometry_msgs::Pose currentLocation;
		ros::Publisher robotNodeStagePub;
		ros::Subscriber stageOdoSub;
		std::queue<geometry_msgs::Point> locationQueue;

		void stage0domCallback(const nav_msgs::Odometry msg);
		void updateDesiredLocationCallback(const geometry_msgs::Point location);
		bool doubleEquals(double a, double b, double difference);
		double normalizeAngle(double angle);
		bool turnAnticlockwise(double currentAngle, double desiredAngle);
		bool atDesiredLocation();
		void updateCurrentVelocity();
		

	private:

};

#endif