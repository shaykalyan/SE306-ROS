#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "DiceRoller.h"
#include "EntertainmentDiceRoller.h"
#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"

EntertainmentDiceRoller::~EntertainmentDiceRoller() {
}

// Signatures
ros::Publisher diceTriggerPub;
elderly_care_simulation::DiceRollTrigger diceRollTrigger;

int main(int argc, char **argv) {
	
    // ROS initialiser calls
    ros::init(argc, argv, "EntertainmentDiceRoller");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);

    // Declare publishers
    diceTriggerPub = nodeHandle.advertise<elderly_care_simulation::DiceRollTrigger>("dice_roll_trigger", 1000, true);

    // Create diceroller
    DiceRoller roller = EntertainmentDiceRoller();

    int tick = 1;

    while (ros::ok()) {
    
        // Every 10 ticks ...        
        if (tick % 10 == 0) {

            // ... roll dice
            bool result = roller.roll();
            if (result) {
                ROS_INFO("YOUR ENTERTAINMENT/AMUSEMENT NEEDS REPLENISHING.");
                diceRollTrigger.type = ENTERTAINMENT;
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
