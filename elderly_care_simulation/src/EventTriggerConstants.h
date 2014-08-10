#ifndef EVENT_TRIGGER_MSG_INCLUDE
#define EVENT_TRIGGER_MSG_INCLUDE 1

const int EVENT_TRIGGER_MSG_TYPE_REQUEST = 0;
const int EVENT_TRIGGER_MSG_TYPE_RESPONSE = 1;

const int EVENT_TRIGGER_EVENT_TYPE_VISITOR = 0;
const int EVENT_TRIGGER_EVENT_TYPE_ASSISTANT = 1;

const int EVENT_TRIGGER_RESULT_FAILURE = 0;
const int EVENT_TRIGGER_RESULT_SUCCESS = 1;

// This is not included in the message definition, but is related to 
// event types (used by the resident to keep track of the current event 
// they're accepting)
const int NO_CURRENT_TASK = -1;

#endif
