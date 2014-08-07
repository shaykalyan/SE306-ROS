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

// ID of tasks to accept
const int ID = 0;

// Used to manage task execution
const int TASK_DURATION = 50;
int timer = 0;
bool isPerformingTask = false;

// Signatures
ros::Publisher stagePub;
ros::Subscriber stageOdoSub;
ros::Subscriber taskSub;
void startTask();
void stopTask();
void stageOdomCallback(nav_msgs::Odometry msg);
void taskRequestCallback(elderly_care_simulation::Task msg);

/**
    Start performing entertainment (spin on the spot).
*/
void startTask() {
    angularZ = 1;
    timer = TASK_DURATION;
    isPerformingTask = true;
}

/**
    Stop spinning.
*/
void stopTask() {
    angularZ = 0;
    isPerformingTask = false;
}

/**
    Receive current position from stage, then update state.
*/
void stageOdomCallback(nav_msgs::Odometry msg) {	
	px = 0 + msg.pose.pose.position.x;
	py = -6 + msg.pose.pose.position.y;
}

/**
    Receive task request. Check if task belongs to this robot,
    then perform task accordingly. 
*/
void taskRequestCallback(elderly_care_simulation::Task msg) {   
    if (msg.type == ID) {
        ROS_INFO("You want entertainment? I'LL GIVE YOU ENTERTAINMENT!");
        startTask();
    }
}

int main(int argc, char **argv) {

	// Set initial pose (same as world file)
	ptheta = M_PI/2.0;
	px = 0;
	py = -6;
	
	// Set initial velocity
	linearX = 0;
	angularZ = 0;
	
    // ROS initialiser calls
    ros::init(argc, argv, "EntertainmentAssistant");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // Declare publishers
    stagePub = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel", 1000); 

    // Declare subscribers
    stageOdoSub = n.subscribe<nav_msgs::Odometry>("robot_1/odom", 1000, stageOdomCallback);
    taskSub = n.subscribe<elderly_care_simulation::Task>("task_request", 1000, taskRequestCallback);

    // Declare outgoing messages
    geometry_msgs::Twist cmdVel;

    while (ros::ok()) {
	    // Publish velocity to Stage
	    cmdVel.linear.x = linearX;
	    cmdVel.angular.z = angularZ;
	    stagePub.publish(cmdVel);
        
        // Check if robot should stop giving entertainment
        if (isPerformingTask) {
            timer--;
            if (timer <= 0) {
                stopTask();
            }
        }

	    ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;

}
