#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"

#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"
#include "EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <unistd.h> // sleep

// Velocity
double linearX;
double angularZ;

// Pose
const double WORLD_POS_X = 0;
const double WORLD_POS_Y = 0;
double px;
double py;
double theta;

// Signatures
ros::Publisher robotNodeStagePub;
ros::Subscriber stageOdoPub;
ros::Subscriber diceTriggerSub;
ros::Publisher residentEvent_pub;
void stageOdomCallback(nav_msgs::Odometry msg);
void diceTriggerCallback();

/**
    Process odometry messages from Stage
*/
void stageOdomCallback(nav_msgs::Odometry msg) {
	
	px = WORLD_POS_X + msg.pose.pose.position.x;
	py = WORLD_POS_Y + msg.pose.pose.position.y;
	// ROS_INFO("Current x position is: %f", px);
	// ROS_INFO("Current y position is: %f", py);
}

void diceTriggerCallback(elderly_care_simulation::DiceRollTrigger msg) {
    elderly_care_simulation::EventTrigger msg_out;
    msg_out.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
    msg_out.result = EVENT_TRIGGER_RESULT_FAILURE;

    switch(msg.type) {
        case MORAL_SUPPORT:
            ROS_INFO("I really need moral support right now ...");
            msg_out.event_type = EVENT_TRIGGER_EVENT_TYPE_VISITOR;
            break;
    }

    ROS_INFO("Sending request to scheduler");
    residentEvent_pub.publish(msg_out);
}

/**
    Process 
*/

int main(int argc, char **argv) {

    // ROS initialiser calls
    ros::init(argc, argv, "Resident");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    // Initialise pose (must be same as world file)
	theta = M_PI/2.0;
	px = WORLD_POS_X;
	py = WORLD_POS_Y;
	
	// Initialise velocity
	linearX = 0.0;
	angularZ = 0.0;
	
    // Initialise publishers
    robotNodeStagePub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 
    residentEvent_pub = n.advertise<elderly_care_simulation::EventTrigger>("resident_event",1000, true);

    // Initialise subscribers
    stageOdoPub = n.subscribe<nav_msgs::Odometry>("robot_0/odom", 1000, stageOdomCallback);
    diceTriggerSub = n.subscribe<elderly_care_simulation::DiceRollTrigger>("dice_roll_trigger", 1000, diceTriggerCallback);

    // Initialise messages
    geometry_msgs::Twist robotNodeCmdvel;

    while (ros::ok()) {

	    // Publish to Stage
	    robotNodeCmdvel.linear.x = linearX;
	    robotNodeCmdvel.angular.z = angularZ;
	    robotNodeStagePub.publish(robotNodeCmdvel);
	
	    ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;

}
