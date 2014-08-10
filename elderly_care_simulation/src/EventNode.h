#include "elderly_care_simulation/EventTrigger.h"

class EventNode {
public:
	EventNode(int priority, elderly_care_simulation::EventTrigger msg);
	int getPriority() const;
	elderly_care_simulation::EventTrigger getEventTriggerMessage() const;
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

elderly_care_simulation::EventTrigger EventNode::getEventTriggerMessage() const {
	return _msg;
}

bool operator<(EventNode lhs, EventNode rhs) {
	return lhs.getPriority() > rhs.getPriority();
}