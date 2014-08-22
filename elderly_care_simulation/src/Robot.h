#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"

#include <sstream>
#include "math.h"
#include <cstdlib>

#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <queue>

class Robot{

	public:
		Robot();
		~Robot();

		ros::ServiceClient pathFinderService;
		ros::Publisher robotNodeStagePub;
		ros::Subscriber stageOdoSub;

		geometry_msgs::Pose currentLocation;
        geometry_msgs::Twist currentVelocity;        

		void stage0domCallback(const nav_msgs::Odometry msg);
		void goToLocation(const geometry_msgs::Point location);
		void startSpinning(bool clockwise);
		void stopSpinning();
		void updateCurrentVelocity();
		bool atDesiredLocation();

	protected:
		std::queue<geometry_msgs::Point> locationQueue;
		double  currentAngle;

		enum Spin {
			NOT_SPINNING, CLOCKWISE, ANTI_CLOCKWISE
		};

		Spin spin;

		void addPointsToQueue(const std::vector<geometry_msgs::Point> points);
		void updateCurrentVelocityToDesiredLocation();
		void clearLocationQueue();
		bool doubleEquals(double a, double b, double difference);
		double normalizeAngle(double angle);
		bool turnAnticlockwise(double currentAngle, double desiredAngle);
};

#endif
