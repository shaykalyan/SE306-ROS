#ifndef MORAL_SUPPORT_DICE_ROLLER_H
#define MORAL_SUPPORT_DICE_ROLLER_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"

class MoralSupportDiceRoller {

    public:

        MoralSupportDiceRoller();
        ~MoralSupportDiceRoller();

        bool roll();

        const int DICE_SIDES = 300;
        int threshold;
};

#endif
