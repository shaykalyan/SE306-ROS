#ifndef ENTERTAINMENT_DICE_ROLLER_H
#define ENTERTAINMENT_DICE_ROLLER_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"

class EntertainmentDiceRoller {

    public:

        EntertainmentDiceRoller();
        ~EntertainmentDiceRoller();

        bool roll();

        const int DICE_SIDES = 200;
        int threshold;
};

#endif


