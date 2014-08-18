#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "DiceRoller.h"
#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"

DiceRoller::DiceRoller(int sides) {
    this->DICE_SIDES = sides;
    this->threshold = DICE_SIDES;
}

DiceRoller::~DiceRoller() {
}

/**
 * Rolls the dice. If the roll meets the threshold,
 * the threshold is reset to the worst probability
 * and true is returned, otherwise probability is 
 * increased and false is returned.
 */
bool DiceRoller::roll() {

    int rolled = rand() % DICE_SIDES + 1;
    ROS_INFO("Rolled: %d. Needed: %d", rolled, threshold);
    
    if (rolled >= threshold) {
        // Reset threshold
        threshold = DICE_SIDES;
        return true;
    } 
    else {
        threshold--;
        return false;
    }   
}
