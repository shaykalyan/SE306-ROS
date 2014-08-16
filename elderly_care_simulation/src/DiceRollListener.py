#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('elderly_care_simulation')
from elderly_care_simulation.msg import DiceRollTrigger

def dice_roll_callback(msg):
    rospy.loginfo("Dice Type: %d Threshold: %4d Rolled: %4d",msg.type, msg.threshold, msg.rolled)
    
def listener():

    # initialise listener node
    # anonymous ensures a unique listeners allowing multiple instances
    # of DiceRollListener to running simultanously
    rospy.init_node('DiceRollListener', anonymous=True)

    # subscribe to topic
    rospy.Subscriber("dice_roll", DiceRollTrigger, dice_roll_callback)

    # prevent python from exiting
    rospy.spin()
        
if __name__ == '__main__':
    listener()