#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"
#include "EventTriggerConstants.h"
#include "elderly_care_simulation/EventTrigger.h"
#include <queue>
#include <vector>
#include <utility>

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
const double WORLD_POS_X = 10;
const double WORLD_POS_Y = 10;
double px;
double py;
double theta;

// 0 = VISITOR // 1 = ASSISTANT
bool robot_switch = true;
bool readyToSend = true;


class EventNode {
public:
	EventNode(int priority, elderly_care_simulation::EventTrigger msg);
	int getPriority() const;
	elderly_care_simulation::EventTrigger getEventTriggerObject() const;
private:
	int _priority;
	elderly_care_simulation::EventTrigger _msg;
};

EventNode::EventNode(int priority, elderly_care_simulation::EventTrigger msg) {
	_priority = priority;
	_msg = msg;
}

int EventNode::getPriority() const {
	return _priority;
}

elderly_care_simulation::EventTrigger EventNode::getEventTriggerObject() const {
	return _msg;
}

bool operator<(EventNode lhs, EventNode rhs) {
	return lhs.getPriority() > rhs.getPriority();
}

struct EventPriorityComparator : public std::binary_function<EventNode*, EventNode*, bool> {
	bool operator() (const EventNode* a, const EventNode* b){
		return a->getPriority() > b->getPriority(); 
	}
};

void EventTrigger_callback(elderly_care_simulation::EventTrigger msg) {
	
	if (msg.msg_type == EVENT_TRIGGER_MSG_TYPE_RESPONSE) {
		if(msg.result == EVENT_TRIGGER_RESULT_SUCCESS){
			if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_VISITOR) {
				ROS_INFO("Response from Visitor");
			} else if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_ASSISTANT) {
				ROS_INFO("Response from Assitant");
			}
			// reset ability to send
			readyToSend = true;
		}
	}
}

int main(int argc, char **argv) {

 	//initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = M_PI/2.0;
	px = WORLD_POS_X;
	py = WORLD_POS_Y;
	
	//Initial velocity
	linear_x = 0.0;
	angular_z = 0.0;

	std::priority_queue<EventNode > eventQueue;

	elderly_care_simulation::EventTrigger m1;
	m1.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
	m1.event_type = EVENT_TRIGGER_EVENT_TYPE_VISITOR;
	m1.result = EVENT_TRIGGER_RESULT_FAILURE;

	elderly_care_simulation::EventTrigger m2;
	m2.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
	m2.event_type = EVENT_TRIGGER_EVENT_TYPE_ASSISTANT;
	m2.result = EVENT_TRIGGER_RESULT_FAILURE;

	elderly_care_simulation::EventTrigger m3;
	m3.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
	m3.event_type = EVENT_TRIGGER_EVENT_TYPE_VISITOR;
	m3.result = EVENT_TRIGGER_RESULT_FAILURE;

	elderly_care_simulation::EventTrigger m4;
	m4.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
	m4.event_type = EVENT_TRIGGER_EVENT_TYPE_VISITOR;
	m4.result = EVENT_TRIGGER_RESULT_FAILURE;

	elderly_care_simulation::EventTrigger m5;
	m5.msg_type = EVENT_TRIGGER_MSG_TYPE_REQUEST;
	m5.event_type = EVENT_TRIGGER_EVENT_TYPE_ASSISTANT;
	m5.result = EVENT_TRIGGER_RESULT_FAILURE;

	eventQueue.push(EventNode(1, m1));
	eventQueue.push(EventNode(2, m2));
	eventQueue.push(EventNode(3, m3));
	eventQueue.push(EventNode(4, m4));
	eventQueue.push(EventNode(5, m5));
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Scheduler");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	// advertise to event_trigger topic
	ros::Publisher EventTrigger_pub = n.advertise<elderly_care_simulation::EventTrigger>("event_trigger",1000, true);

	// subscribe to event_trigger topic
	ros::Subscriber EventTrigger_sub = n.subscribe<elderly_care_simulation::EventTrigger>("event_trigger",1000, EventTrigger_callback);

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;

	while (ros::ok()) {

		if (readyToSend) {

			if(eventQueue.size() <= 0) {
				ROS_INFO("No more tasks to do.");
				continue;
			}

			// block scheduler
			readyToSend = false;

			elderly_care_simulation::EventTrigger msg;
			msg = eventQueue.top().getEventTriggerObject();
			eventQueue.pop();

			if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_VISITOR) {
				ROS_INFO("Publishing to Visitor");
			} else if (msg.event_type == EVENT_TRIGGER_EVENT_TYPE_ASSISTANT) {
				ROS_INFO("Publishing to Assitant");
			}
			
			EventTrigger_pub.publish(msg);
		}

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
