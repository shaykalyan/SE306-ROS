#ifndef GUI_COMM_MSG_INCLUDE
#define GUI_COMM_MSG_INCLUDE 1

// ========================================
// =           MESSAGE TYPE               =
// ========================================
const int GUI_COMM_MSG_TYPE_UNDEFINED = 0;
const int GUI_COMM_MSG_TYPE_REQUEST   = 1;
const int GUI_COMM_MSG_TYPE_INFO      = 2;

// ========================================
// =               ACTION                 =
// ========================================
const int GUI_COMM_ACTION_UNDEFINED  = 0;
const int GUI_COMM_ACTION_POPULATE  = 1;
const int GUI_COMM_ACTION_CLEAR     = 2;

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
