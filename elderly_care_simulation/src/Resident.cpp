#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"

#include "elderly_care_simulation/Task.h"
#include "elderly_care_simulation/DiceRollResult.h"
#include "elderly_care_simulation/DiceRollRequest.h"

// Velocity
double linearX;
double angularZ;

// Pose
double px;
double py;
double ptheta;

// Signatures
ros::Publisher stagePub;
ros::Publisher taskPub;
ros::Publisher diceRollPub;
ros::Subscriber stageOdoSub;
ros::Subscriber diceResultSub;
void stageOdomCallback(nav_msgs::Odometry msg);
void diceResultCallback(elderly_care_simulation::DiceRollResult msg);

/**
    Receive current position from stage, then update state.
*/
void stageOdomCallback(nav_msgs::Odometry msg) {	
	px = 0 + msg.pose.pose.position.x;
	py = -4 + msg.pose.pose.position.y;
}

/**
    Receive result of moral support dice roll. 
*/
void diceResultCallback(elderly_care_simulation::DiceRollResult msg) {
    if (msg.result) {
        ROS_INFO("WELP. I GUESS I NEED MORAL SUPPORT.");
    }
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
    stagePub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 1000);
    taskPub = n.advertise<elderly_care_simulation::Task>("task_request", 1000);
    diceRollPub = n.advertise<elderly_care_simulation::DiceRollRequest>("diceroll_request", 1000);

    // Declare subscribers
    stageOdoSub = n.subscribe<nav_msgs::Odometry>("robot_0/odom", 1000, stageOdomCallback);
    diceResultSub = n.subscribe<elderly_care_simulation::DiceRollResult>("diceroll_result", 1000, diceResultCallback);

    // Declare outgoing messages
    geometry_msgs::Twist cmdVel;
    elderly_care_simulation::Task taskRequest;
    elderly_care_simulation::DiceRollRequest diceRollRequest;
    
    int count = 0;

    while (ros::ok()) {
	    // Publish velocity to Stage
	    cmdVel.linear.x = linearX;
	    cmdVel.angular.z = angularZ;
	    stagePub.publish(cmdVel);

        // ALPHA: Every 10 seconds, request entertainment
        count++;
        if (count == 100) {
            ROS_INFO("I'M BORED. GIVE ME ENTERTAINMENT.");
            taskRequest.type = 0;
            taskRequest.id = 12345;
            taskPub.publish(taskRequest);
            count = 0;
        }

        // ALPHA: Initiate a moral support dice roll
        if (count % 10 == 0) {
            ROS_INFO("ROLLING THE DICE FOR MORALE SUPPORT.");
            diceRollRequest.type = 0;
            diceRollPub.publish(diceRollRequest);
        }

	    ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;

}
