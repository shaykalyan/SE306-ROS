#include "elderly_care_simulation/EventTrigger.h"

class EventNode {
public:
	EventNode(int priority, elderly_care_simulation::EventTrigger msg);
	int getPriority() const;
	int getWeight() const;
	elderly_care_simulation::EventTrigger getEventTriggerMessage() const;
private:
	static int _count;
	int _weight;
	int _priority;
	elderly_care_simulation::EventTrigger _msg;
};

int EventNode::_count = 0;

EventNode::EventNode(int priority, elderly_care_simulation::EventTrigger msg) {
	_priority = priority;
	_msg = msg;
	_weight = EventNode::_count;
	EventNode::_count++;
}

int EventNode::getPriority() const {
	return _priority;
}

int EventNode::getWeight() const {
	return _weight;
}

elderly_care_simulation::EventTrigger EventNode::getEventTriggerMessage() const {
	return _msg;
}

bool operator<(EventNode lhs, EventNode rhs) {
	if (lhs.getPriority() == rhs.getPriority()){
		return lhs.getWeight() > rhs.getWeight();
	}
	return lhs.getPriority() > rhs.getPriority();
}