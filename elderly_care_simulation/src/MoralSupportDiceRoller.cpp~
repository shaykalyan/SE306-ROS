#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>
#include "math.h"

#include "elderly_care_simulation/DiceRollRequest.h"
#include "elderly_care_simulation/DiceRollResult.h"

// ID of dice roll requests to accept
const int ID = 0;

// Number of sides this dice has
const int DICE_SIDES = 100;
int threshold = DICE_SIDES;

// Signatures
ros::Publisher diceResultPub;
ros::Subscriber diceRollSub;
elderly_care_simulation::DiceRollResult diceRollResult;
void diceRollRequestCallback(elderly_care_simulation::DiceRollRequest msg);

void diceRollRequestCallback(elderly_care_simulation::DiceRollRequest msg) {
    
    // Only perform roll request if it belongs to this node
    if (msg.type == ID) {

        int rolled = rand() % DICE_SIDES + 1;
        ROS_INFO("ROLLED: %d. NEEDED: %d", rolled, threshold);

        if (rolled >= threshold) {
            ROS_INFO("YOUR MORALE NEEDS REPLENISHING.");
            diceRollResult.result = true;
            diceResultPub.publish(diceRollResult);

            // Reset threshold
            threshold = DICE_SIDES;
        }
        else {
            diceRollResult.result = false;
            diceResultPub.publish(diceRollResult);

            // Make the next roll more likely to succeed
            threshold--;
        }
    }
}

int main(int argc, char **argv) {
	
    // ROS initialiser calls
    ros::init(argc, argv, "MoralSupportDiceRoller");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // Declare publishers
    diceResultPub = n.advertise<elderly_care_simulation::DiceRollResult>("diceroll_result", 1000);

    // Declare subscribers
    diceRollSub = n.subscribe<elderly_care_simulation::DiceRollRequest>("diceroll_request", 1000, diceRollRequestCallback);

    while (ros::ok()) {
	    ros::spin();
	    loop_rate.sleep();
    }

    return 0;

}
