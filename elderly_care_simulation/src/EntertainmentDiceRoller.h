#ifndef ENTERTAINMENT_DICE_ROLLER_H
#define ENTERTAINMENT_DICE_ROLLER_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "DiceRoller.h"
#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"

class EntertainmentDiceRoller : public DiceRoller {

    public:

        EntertainmentDiceRoller() : DiceRoller(200) {}
        ~EntertainmentDiceRoller();
};

#endif


