#ifndef GUI_COMM_MSG_INCLUDE
#define GUI_COMM_MSG_INCLUDE 1

// ========================================
// =           MESSAGE TYPE               =
// ========================================
const int GUI_COMM_MSG_TYPE_REQUEST = 0;
const int GUI_COMM_MSG_TYPE_INFO    = 1;

// ========================================
// =               ACTION                 =
// ========================================
const int GUI_COMM_ACTION_POPULATE  = 0;
const int GUI_COMM_ACTION_CLEAR     = 1;

/**
 * Returns a C string representation of the corresponding action type
 */
const char * actionToString(int action) {
    switch(action){
        case GUI_COMM_ACTION_POPULATE:                  return "POPULATE";
        case GUI_COMM_ACTION_CLEAR:                     return "CLEAR";
        default:                                        return "UNDEFINED";
    }
}

#endif
