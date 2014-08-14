#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"

// Number of sides this dice has
const int DICE_SIDES = 200;

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

    int tick = 1;
    int threshold = DICE_SIDES;

    while (ros::ok()) {
        
        // Roll dice every 10 ticks
        if (tick % 10 == 0) {

            int rolled = rand() % DICE_SIDES + 1;
            //ROS_INFO("Rolled: %d. Needed: %d", rolled, threshold);
            
            if (rolled >= threshold) {

                ROS_INFO("Resident Requests Entertainment");
                diceRollTrigger.type = ENTERTAINMENT;
                diceTriggerPub.publish(diceRollTrigger);

                // Reset threshold
                threshold = DICE_SIDES;
            } 
            else {

                threshold--;
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
