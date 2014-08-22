#ifndef DICE_ROLLER_H
#define DICE_ROLLER_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"

class DiceRoller {

    public:

        DiceRoller(int sides);
        ~DiceRoller();

        bool roll();

        int threshold;

    private:

        int DICE_SIDES; // not const as it needs to be assigned in constructor (work-around)
};

#endif


