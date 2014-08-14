#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "MoralSupportDiceRoller.h"
#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"

MoralSupportDiceRoller::MoralSupportDiceRoller() {
    threshold = DICE_SIDES;
}

MoralSupportDiceRoller::~MoralSupportDiceRoller() {
}

/**
 * Rolls the dice. If the roll meets the threshold,
 * the threshold is reset to the worst probability
 * and true is returned, otherwise probability is 
 * increased and false is returned.
 */
bool MoralSupportDiceRoller::roll() {

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

// Signatures
ros::Publisher diceTriggerPub;
elderly_care_simulation::DiceRollTrigger diceRollTrigger;

int main(int argc, char **argv) {
	
    // ROS initialiser calls
    ros::init(argc, argv, "MoralSupportDiceRoller");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);

    // Declare publishers
    diceTriggerPub = nodeHandle.advertise<elderly_care_simulation::DiceRollTrigger>("dice_roll_trigger", 1000, true);

    // Create diceroller
    MoralSupportDiceRoller roller = MoralSupportDiceRoller();

    int tick = 1;

    while (ros::ok()) {
        
        if (tick % 10 == 0) {

            // Roll dice
            bool result = roller.roll();
            if (result) {
                ROS_INFO("YOUR MORALE NEEDS REPLENISHING.");
                diceRollTrigger.type = MORAL_SUPPORT;
                diceTriggerPub.publish(diceRollTrigger);
            } 
            tick = 1;
        }
        else {
            tick++;
        }  
        
	    ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;
}
