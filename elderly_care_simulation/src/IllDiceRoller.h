#ifndef ILL_DICE_ROLLER_H
#define ILL_DICE_ROLLER_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "DiceRoller.h"
#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"

class IllDiceRoller : public DiceRoller {

    public:

        IllDiceRoller() : DiceRoller(5000) {}
        ~IllDiceRoller();
};

#endif


