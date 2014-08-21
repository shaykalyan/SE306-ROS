#ifndef VERY_ILL_DICE_ROLLER_H
#define VERY_ILL_DICE_ROLLER_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "DiceRoller.h"
#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"

class VeryIllDiceRoller : public DiceRoller {

    public:

        VeryIllDiceRoller() : DiceRoller(10000) {}
        ~VeryIllDiceRoller();
};

#endif


