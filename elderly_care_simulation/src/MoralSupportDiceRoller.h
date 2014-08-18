#ifndef MORAL_SUPPORT_DICE_ROLLER_H
#define MORAL_SUPPORT_DICE_ROLLER_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "DiceRoller.h"
#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"

class MoralSupportDiceRoller : public DiceRoller {

    public:

        MoralSupportDiceRoller() : DiceRoller(300) {}
        ~MoralSupportDiceRoller();
};

#endif


