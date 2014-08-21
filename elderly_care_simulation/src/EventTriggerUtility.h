#ifndef EVENT_TRIGGER_MSG_INCLUDE
#define EVENT_TRIGGER_MSG_INCLUDE 1

// ========================================
// =           MESSAGE TYPE               =
// ========================================
const int EVENT_TRIGGER_MSG_TYPE_UNDEFINED = 0;
const int EVENT_TRIGGER_MSG_TYPE_REQUEST = 1;
const int EVENT_TRIGGER_MSG_TYPE_RESPONSE = 2;

// ========================================
// =            EVENT TYPE                =
// ========================================
// VISITOR (CARE GIVERS)
const int EVENT_TRIGGER_EVENT_TYPE_UNDEFINED            = 0;
const int EVENT_TRIGGER_EVENT_TYPE_EAT                  = 1;
const int EVENT_TRIGGER_EVENT_TYPE_SHOWER               = 2;
const int EVENT_TRIGGER_EVENT_TYPE_EXERCISE             = 3;
const int EVENT_TRIGGER_EVENT_TYPE_CONVERSATION         = 4;
const int EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT        = 5; // RAND

// VISITOR (FRIEND & RELATIVE)
const int EVENT_TRIGGER_EVENT_TYPE_FRIEND_RELATIVE      = 6;

// VISITOR (DOCTOR & NURSE)
const int EVENT_TRIGGER_EVENT_TYPE_ILL                  = 7;
const int EVENT_TRIGGER_EVENT_TYPE_VERY_ILL             = 8;

// ASSISTANT
const int EVENT_TRIGGER_EVENT_TYPE_MEDICATION           = 9;
const int EVENT_TRIGGER_EVENT_TYPE_COOK                 = 10;
const int EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT        = 11;
const int EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP        = 12;

// RESIDENT
const int EVENT_TRIGGER_EVENT_TYPE_WAKE                 = 13;
const int EVENT_TRIGGER_EVENT_TYPE_SLEEP                = 14;

// ========================================
// =           EVENT PRIORITY             =
// ========================================
const int EVENT_TRIGGER_PRIORITY_VERY_HIGH               = 0;
const int EVENT_TRIGGER_PRIORITY_HIGH                    = 1;
const int EVENT_TRIGGER_PRIORITY_MEDIUM                  = 2;
const int EVENT_TRIGGER_PRIORITY_LOW                     = 3;
const int EVENT_TRIGGER_PRIORITY_VERY_LOW                = 4;
const int EVENT_TRIGGER_PRIORITY_NONE                    = 5;
const int EVENT_TRIGGER_PRIORITY_UNDEFINED               = 6;

// ========================================
// =           EVENT WEIGHT               =
// ========================================
const int EVENT_TRIGGER_WEIGHT_0                         = 0;
const int EVENT_TRIGGER_WEIGHT_1                         = 1;
const int EVENT_TRIGGER_WEIGHT_2                         = 2;
const int EVENT_TRIGGER_WEIGHT_3                         = 3;
const int EVENT_TRIGGER_WEIGHT_4                         = 4;
const int EVENT_TRIGGER_WEIGHT_5                         = 5;
const int EVENT_TRIGGER_WEIGHT_UNDEFINED                 = 6;

// ========================================
// =              RESULT                  =
// ========================================
const int EVENT_TRIGGER_RESULT_UNDEFINED = 0;
const int EVENT_TRIGGER_RESULT_FAILURE = 1;
const int EVENT_TRIGGER_RESULT_SUCCESS = 2;

// This is not included in the message definition, but is related to 
// event types (used by the resident to keep track of the current event 
// they're accepting)
const int NO_CURRENT_TASK = -1;

// ========================================
// =          RELAVENT FUNCTIONS          =
// ========================================
/**
 * Returns a C string representation of the corresponding event type
 */
const char * eventTypeToString(int eventType) {
    switch(eventType){

        case EVENT_TRIGGER_EVENT_TYPE_EAT:              return "EAT";
        case EVENT_TRIGGER_EVENT_TYPE_SHOWER:           return "SHOWER";
        case EVENT_TRIGGER_EVENT_TYPE_EXERCISE:         return "EXERCISE";
        case EVENT_TRIGGER_EVENT_TYPE_CONVERSATION:     return "CONVERSATION";
        case EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT:    return "MORAL_SUPPORT";
        case EVENT_TRIGGER_EVENT_TYPE_FRIEND_RELATIVE:  return "FRIEND_RELATIVE";
        case EVENT_TRIGGER_EVENT_TYPE_ILL:              return "ILL";
        case EVENT_TRIGGER_EVENT_TYPE_VERY_ILL:         return "VERY_ILL";
        case EVENT_TRIGGER_EVENT_TYPE_MEDICATION:       return "MEDICATION";
        case EVENT_TRIGGER_EVENT_TYPE_COOK:             return "COOK";
        case EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT:    return "ENTERTAINMENT";
        case EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP:    return "COMPANIONSHIP";
        case EVENT_TRIGGER_EVENT_TYPE_WAKE:             return "WAKE";
        case EVENT_TRIGGER_EVENT_TYPE_SLEEP:            return "SLEEP";
        default:                                        return "UNDEFINED";
    }
}

/**
 * Returns a C string representation of the corresponding priority
 */
const char * priorityToString(int priority) {
    switch(priority){

        case EVENT_TRIGGER_PRIORITY_VERY_HIGH:          return "VERY_HIGH";
        case EVENT_TRIGGER_PRIORITY_HIGH:               return "HIGH";
        case EVENT_TRIGGER_PRIORITY_MEDIUM:             return "MEDIUM";
        case EVENT_TRIGGER_PRIORITY_LOW:                return "LOW";
        case EVENT_TRIGGER_PRIORITY_VERY_LOW:           return "VERY_LOW";
        case EVENT_TRIGGER_PRIORITY_NONE:               return "NONE";
        default:                                        return "UNDEFINED";
    }
}

int getEventWeight(int eventType) {
    switch(eventType){

        case EVENT_TRIGGER_EVENT_TYPE_EAT:              return EVENT_TRIGGER_WEIGHT_2;
        case EVENT_TRIGGER_EVENT_TYPE_SHOWER:           return EVENT_TRIGGER_WEIGHT_2;
        case EVENT_TRIGGER_EVENT_TYPE_SLEEP:            return EVENT_TRIGGER_WEIGHT_2;

        case EVENT_TRIGGER_EVENT_TYPE_EXERCISE:         return EVENT_TRIGGER_WEIGHT_1;
        case EVENT_TRIGGER_EVENT_TYPE_CONVERSATION:     return EVENT_TRIGGER_WEIGHT_1;
        case EVENT_TRIGGER_EVENT_TYPE_MORAL_SUPPORT:    return EVENT_TRIGGER_WEIGHT_1;
        case EVENT_TRIGGER_EVENT_TYPE_FRIEND_RELATIVE:  return EVENT_TRIGGER_WEIGHT_1;
        case EVENT_TRIGGER_EVENT_TYPE_ILL:              return EVENT_TRIGGER_WEIGHT_1;
        case EVENT_TRIGGER_EVENT_TYPE_VERY_ILL:         return EVENT_TRIGGER_WEIGHT_1;
        case EVENT_TRIGGER_EVENT_TYPE_MEDICATION:       return EVENT_TRIGGER_WEIGHT_1;
        case EVENT_TRIGGER_EVENT_TYPE_ENTERTAINMENT:    return EVENT_TRIGGER_WEIGHT_1;
        case EVENT_TRIGGER_EVENT_TYPE_COMPANIONSHIP:    return EVENT_TRIGGER_WEIGHT_1;

        case EVENT_TRIGGER_EVENT_TYPE_COOK:             return EVENT_TRIGGER_WEIGHT_0;
        case EVENT_TRIGGER_EVENT_TYPE_WAKE:             return EVENT_TRIGGER_WEIGHT_0;

        default:                                        return EVENT_TRIGGER_WEIGHT_UNDEFINED;
    }
}

#endif
