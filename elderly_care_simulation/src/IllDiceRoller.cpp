#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "DiceRoller.h"
#include "IllDiceRoller.h"
#include "DiceRollerTypeConstants.h"
#include "elderly_care_simulation/DiceRollTrigger.h"
#include "elderly_care_simulation/DiceRollReport.h"


IllDiceRoller::~IllDiceRoller() {
}

// Signatures
ros::Publisher diceTriggerPub;
ros::Publisher rollReportPub;
elderly_care_simulation::DiceRollTrigger diceRollTrigger;
elderly_care_simulation::DiceRollReport diceRollReport;

int main(int argc, char **argv) {
	
    // ROS initialiser calls
    ros::init(argc, argv, "IllDiceRoller");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);

    // Declare publishers
    diceTriggerPub = nodeHandle.advertise<elderly_care_simulation::DiceRollTrigger>("dice_roll_trigger", 1000, true);
    rollReportPub = nodeHandle.advertise<elderly_care_simulation::DiceRollReport>("dice_roll_report", 1000, true);

    // Create diceroller
    IllDiceRoller roller = IllDiceRoller();

    int tick = 1;

    while (ros::ok()) {
        
        // Every 30 ticks ...
        if (tick % 30 == 0) {

            // ... roll dice
            bool result = roller.roll();
            if (result) {
                ROS_INFO("YOU HAVE CAUGHT A MINOR ILLNESS.");
                diceRollTrigger.type = ILL;
                diceTriggerPub.publish(diceRollTrigger);
            } 
            tick = 1;
        }
        else {
            tick++;
        }

        // Publish report of roll
        diceRollReport.threshold = roller.oldThreshold;
        diceRollReport.rolled = roller.rolled;
        diceRollReport.numSides = roller.DICE_SIDES;
        diceRollReport.type = ILL;
        rollReportPub.publish(diceRollReport);  
        
	    ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;
}
