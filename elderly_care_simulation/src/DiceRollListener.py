#!/usr/bin/env python
import rospy
import roslib
import roslib.load_manifest('elderly_care_simulation')
from elderly_care_simulation.msg import DiceRollTrigger
from Tkinter import *

class DiceRollerGUI:
    """
    TkInter GUI for the visualisation of dice rolls carried out by the 
    various *DiceRoller implementations
    """
    def __init__(self):
        """
        Initialises tkInter elements and subscribes to relevant ROS Topics
        """
        # create root element with fixed size
        self.root = Tk()
        self.root.geometry('400x200+1+1')

        # create frame element to host labels and graphics
        self.frame = Frame(self.root)        
        self.frame.pack()

        # create variable label
        self.dice_label = StringVar()

        # create and assign dice label to label widget. Updating dice_label will
        # automatically update the widget's text
        self.dice_label_widget = Label(self.frame, textvariable=self.dice_label)
        self.dice_label_widget.pack(side=LEFT)

        # initialise listener node
        # anonymous ensures a unique listeners allowing multiple instances
        # of DiceRollListener to running simultanously
        rospy.init_node('DiceRollListener', anonymous=True)

        # subscribe to topic
        rospy.Subscriber("dice_roll", DiceRollTrigger, self.dice_roll_callback)


    def run(self):
        """
        Initiate tkInter's main loop. This will populate the root window
        with widgets as declared in __init__
        """
        self.root.mainloop()

    def update_dice_label(self, data):
        self.dice_label.set(data)

    def dice_roll_callback(self, msg):
        data = 'Dice Type: %d Threshold: %4d Rolled: %4d' % (msg.type, msg.threshold, msg.rolled)
        # rospy.loginfo("Dice Type: %d Threshold: %4d Rolled: %4d",msg.type, msg.threshold, msg.rolled)
        self.update_dice_label(data)

if __name__=='__main__':
    gui = DiceRollerGUI()
    gui.run()