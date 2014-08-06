#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"

#include "elderly_care_simulation/Task.h"

// Velocity
double linearX;
double angularZ;

// Pose
double px;
double py;
double ptheta;

/**
    Receive current position from stage, then update state.
*/
void stageOdomCallback(nav_msgs::Odometry msg) {	
	px = 0 + msg.pose.pose.position.x;
	py = -4 + msg.pose.pose.position.y;
}

int main(int argc, char **argv) {
	// Set initial pose (same as world file)
	ptheta = M_PI/2.0;
	px = 0;
	py = -4;
	
	// Set initial velocity
	linearX = 0;
	angularZ = 0;
	
    // ROS initialiser calls
    ros::init(argc, argv, "Resident");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // Declare publishers
    ros::Publisher stagePub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 1000);
    ros::Publisher taskPub = n.advertise<elderly_care_simulation::Task>("task_request", 1000);

    // Declare subscribers
    ros::Subscriber stageOdoSub = n.subscribe<nav_msgs::Odometry>("robot_0/odom", 1000, stageOdomCallback);

    // Declare outgoing messages
    geometry_msgs::Twist cmdVel;
    elderly_care_simulation::Task taskRequest;
    
    int count = 0;

    while (ros::ok()) {
	    // Publish velocity to Stage
	    cmdVel.linear.x = linearX;
	    cmdVel.angular.z = angularZ;
	    stagePub.publish(cmdVel);

        // ALPHA: Every 10 seconds, request entertainment
        count++;
        if (count == 100) {
            taskRequest.type = 0;
            taskRequest.id = 12345;
            taskPub.publish(taskRequest);
            count = 0;
        }

	    ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;

}
