#ifndef EVENT_TRIGGER_MSG_INCLUDE
#define EVENT_TRIGGER_MSG_INCLUDE 1

const int EVENT_TRIGGER_MSG_TYPE_REQUEST = 0;
const int EVENT_TRIGGER_MSG_TYPE_RESPONSE = 1;

// VISITOR (CARE GIVERS)
const int EVENT_TRIGGER_EVENT_TYPE_EAT                  = 0;
const int EVENT_TRIGGER_EVENT_TYPE_SHOWER               = 1;
const int EVENT_TRIGGER_EVENT_TYPE_EXERCISE             = 2;
const int EVENT_TRIGGER_EVENT_TYPE_CONVERSATION         = 3;
const int EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT        = 4; // RAND

// VISITOR (FRIEND & RELATIVE)
const int EVENT_TRIGGER_EVENT_TYPE_FRIEND_RELATIVE      = 5;

// VISITOR (DOCTOR & NURSE)
const int EVENT_TRIGGER_EVENT_TYPE_ILL                  = 6;
const int EVENT_TRIGGER_EVENT_TYPE_VERY_ILL             = 7;

// ASSISTANT
const int EVENT_TRIGGER_EVENT_TYPE_MEDICATION           = 8;
const int EVENT_TRIGGER_EVENT_TYPE_COOK                 = 9;
const int EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT        = 10;
const int EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP        = 11;

// RESIDENT
const int EVENT_TRIGGER_EVENT_TYPE_WAKE                 = 12;
const int EVENT_TRIGGER_EVENT_TYPE_SLEEP                = 13;

// DEPRICATED
const int EVENT_TRIGGER_EVENT_TYPE_VISITOR                 = 14;
const int EVENT_TRIGGER_EVENT_TYPE_ASSISTANT               = 15;


const int EVENT_TRIGGER_RESULT_FAILURE = 0;
const int EVENT_TRIGGER_RESULT_SUCCESS = 1;

// This is not included in the message definition, but is related to 
// event types (used by the resident to keep track of the current event 
// they're accepting)
const int NO_CURRENT_TASK = -1;

#endif
