#!/usr/bin/env python

# ========================================
# =           MESSAGE TYPE               =
# ========================================

ET_MSG_TYPE_UNDEFINED           = 0;
ET_MSG_TYPE_REQUEST             = 1;
ET_MSG_TYPE_RESPONSE            = 2;

# ========================================
# =            EVENT TYPE                =
# ========================================
ET_EVENT_TYPE_UNDEFINED         = 0;
ET_EVENT_TYPE_EAT               = 1;
ET_EVENT_TYPE_SHOWER            = 2;
ET_EVENT_TYPE_EXERCISE          = 3;
ET_EVENT_TYPE_CONVERSATION      = 4;
ET_EVENT_TYPE_MORAL_SUPPORT     = 5;
ET_EVENT_TYPE_RELATIVE          = 6;
ET_EVENT_TYPE_FRIEND            = 7;
ET_EVENT_TYPE_ILL               = 8;
ET_EVENT_TYPE_VERY_ILL          = 9;
ET_EVENT_TYPE_MEDICATION        = 10;
ET_EVENT_TYPE_COOK              = 11;
ET_EVENT_TYPE_ENTERTAINMENT     = 12;
ET_EVENT_TYPE_COMPANIONSHIP     = 13;
ET_EVENT_TYPE_WAKE              = 14;
ET_EVENT_TYPE_SLEEP             = 15;
ET_EVENT_TYPE_MOVE_TO_KITCHEN   = 16;
ET_EVENT_TYPE_MOVE_TO_BEDROOM   = 17;
ET_EVENT_TYPE_MOVE_TO_HALLWAY   = 18;
ET_EVENT_TYPE_MOVE_TO_TOILET    = 19;

# ========================================
# =           EVENT PRIORITY             =
# ========================================
ET_EVENT_PRIORITY_VERY_HIGH     = 0;
ET_EVENT_PRIORITY_HIGH          = 1;
ET_EVENT_PRIORITY_MEDIUM        = 2;
ET_EVENT_PRIORITY_LOW           = 3;
ET_EVENT_PRIORITY_VERY_LOW      = 4;
ET_EVENT_PRIORITY_NONE          = 5;
ET_EVENT_PRIORITY_UNDEFINED     = 6;

# ========================================
# =           EVENT WEIGHT               =
# ========================================
ET_EVENT_WEIGHT_0               = 0;
ET_EVENT_WEIGHT_1               = 1;
ET_EVENT_WEIGHT_2               = 2;
ET_EVENT_WEIGHT_3               = 3;
ET_EVENT_WEIGHT_4               = 4;
ET_EVENT_WEIGHT_5               = 5;
ET_EVENT_WEIGHT_UNDEFINED       = 6;

# ========================================
# =              RESULT                  =
# ========================================
ET_EVENT_RESULT_UNDEFINED       = 0;
ET_EVENT_RESULT_FAILURE         = 1;
ET_EVENT_RESULT_SUCCESS         = 2;

# ========================================
# =           DICTIONARIES               =
# ========================================
typeToStringDict = {
    
    ET_EVENT_TYPE_UNDEFINED         : 'Undefined',
    ET_EVENT_TYPE_EAT               : 'Eating',
    ET_EVENT_TYPE_SHOWER            : 'Showering',
    ET_EVENT_TYPE_EXERCISE          : 'Exercising',
    ET_EVENT_TYPE_CONVERSATION      : 'Conversation',
    ET_EVENT_TYPE_MORAL_SUPPORT     : 'Moral',
    ET_EVENT_TYPE_RELATIVE          : 'Relative',
    ET_EVENT_TYPE_FRIEND            : 'Friend',
    ET_EVENT_TYPE_ILL               : 'Ill',
    ET_EVENT_TYPE_VERY_ILL          : 'Very ILL',
    ET_EVENT_TYPE_MEDICATION        : 'Medication',
    ET_EVENT_TYPE_COOK              : 'Cooking',
    ET_EVENT_TYPE_ENTERTAINMENT     : 'Entertainment',
    ET_EVENT_TYPE_COMPANIONSHIP     : 'Companionship',
    ET_EVENT_TYPE_WAKE              : 'Wake',
    ET_EVENT_TYPE_SLEEP             : 'Sleep',
    ET_EVENT_TYPE_MOVE_TO_KITCHEN   : 'Kitchen',
    ET_EVENT_TYPE_MOVE_TO_BEDROOM   : 'Bedroom',
    ET_EVENT_TYPE_MOVE_TO_HALLWAY   : 'Hallway',
    ET_EVENT_TYPE_MOVE_TO_TOILET    : 'Toilet'

}

priorityToStringDict = {
    
    ET_EVENT_PRIORITY_VERY_HIGH : 'VERY_HIGH',
    ET_EVENT_PRIORITY_HIGH      : 'HIGH',
    ET_EVENT_PRIORITY_MEDIUM    : 'MEDIUM',
    ET_EVENT_PRIORITY_LOW       : 'LOW',
    ET_EVENT_PRIORITY_VERY_LOW  : 'VERY_LOW',
    ET_EVENT_PRIORITY_NONE      : 'NONE'
}

eventWeightDict = {
    
    ET_EVENT_TYPE_EAT               : ET_EVENT_WEIGHT_5,
    ET_EVENT_TYPE_WAKE              : ET_EVENT_WEIGHT_5,
    ET_EVENT_TYPE_SLEEP             : ET_EVENT_WEIGHT_5,
    ET_EVENT_TYPE_SHOWER            : ET_EVENT_WEIGHT_5,
    ET_EVENT_TYPE_MOVE_TO_KITCHEN   : ET_EVENT_WEIGHT_5,
    ET_EVENT_TYPE_MOVE_TO_BEDROOM   : ET_EVENT_WEIGHT_5,
    ET_EVENT_TYPE_MOVE_TO_HALLWAY   : ET_EVENT_WEIGHT_5,
    ET_EVENT_TYPE_MOVE_TO_TOILET    : ET_EVENT_WEIGHT_5,

    ET_EVENT_TYPE_EXERCISE          : ET_EVENT_WEIGHT_3,
    ET_EVENT_TYPE_CONVERSATION      : ET_EVENT_WEIGHT_3,
    ET_EVENT_TYPE_MORAL_SUPPORT     : ET_EVENT_WEIGHT_3,
    ET_EVENT_TYPE_RELATIVE          : ET_EVENT_WEIGHT_2,
    ET_EVENT_TYPE_FRIEND            : ET_EVENT_WEIGHT_2,
    ET_EVENT_TYPE_MEDICATION        : ET_EVENT_WEIGHT_2,
    ET_EVENT_TYPE_ENTERTAINMENT     : ET_EVENT_WEIGHT_2,
    ET_EVENT_TYPE_COMPANIONSHIP     : ET_EVENT_WEIGHT_2,

    ET_EVENT_TYPE_COOK              : ET_EVENT_WEIGHT_0,
    ET_EVENT_TYPE_ILL               : ET_EVENT_WEIGHT_0,
    ET_EVENT_TYPE_VERY_ILL          : ET_EVENT_WEIGHT_0
    
}

def eventTypeToString(eventType):
    try:
        return typeToStringDict[eventType]
    except KeyError:
        return 'UNDEFINED'

def priorityToString(priority):
    try:
        return priorityToStringDict[priority]
    except KeyError:
        return 'UNDEFINED'

def getEventWeight(eventType):
    try:
        return eventWeightDict[eventType]
    except KeyError:
        return ET_EVENT_WEIGHT_UNDEFINED
