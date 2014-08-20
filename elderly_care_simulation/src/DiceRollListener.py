#!/usr/bin/env python
import rospy
import roslib
import roslib; roslib.load_manifest('elderly_care_simulation')
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
        self.root.geometry('400x250+1+1')

        # create frame element to host labels and graphics
        self.frame = Frame(self.root)        
        self.frame.pack()

        # create variable label
        self.dice_label = StringVar()

        #grid.columnconfigure(frame,0,weight=1)

        # Creates Labels which will eventually contain information about robots and their tasks
        # :Column Names
        Label(self.frame, text="Robot Name", anchor=W, relief=RIDGE, width=15, bg='ivory4').grid(row=0, column=0)
        Label(self.frame, text="Current Task", anchor=W, relief=RIDGE, width=15, bg='ivory4').grid(row=0, column=1)
        Label(self.frame, text="Queued Task", anchor=W, relief=RIDGE, width=15, bg='ivory4').grid(row=0, column=2)

        # : Row 1: Resident
        Label(self.frame, text="Resident", anchor=W, relief=RIDGE, width=15).grid(row=1, column=0)
        Label(self.frame, text="Sleep", anchor=W, relief=RIDGE, width=15).grid(row=1, column=1)
        Label(self.frame, text="None Scheduled", anchor=W, relief=RIDGE, width=15).grid(row=1, column=2)

        # : Row 1: Cook
        Label(self.frame, text="Cook", anchor=W, relief=RIDGE, width=15).grid(row=2, column=0)
        Label(self.frame, text="None", anchor=W, relief=RIDGE, width=15).grid(row=2, column=1)
        Label(self.frame, text="None Scheduled", anchor=W, relief=RIDGE, width=15).grid(row=2, column=2)

        # : Row 1: Medication
        Label(self.frame, text="Medication", anchor=W, relief=RIDGE, width=15).grid(row=3, column=0)
        Label(self.frame, text="None", anchor=W, relief=RIDGE, width=15).grid(row=3, column=1)
        Label(self.frame, text="None Scheduled", anchor=W, relief=RIDGE, width=15).grid(row=3, column=2)

        # : Row 1: Entertainment
        Label(self.frame, text="Entertainment", anchor=W, relief=RIDGE, width=15).grid(row=4, column=0)
        Label(self.frame, text="None", anchor=W, relief=RIDGE, width=15).grid(row=4, column=1)
        Label(self.frame, text="None Scheduled", anchor=W, relief=RIDGE, width=15).grid(row=4, column=2)

        # : Row 1: Companionship
        Label(self.frame, text="Companionship", anchor=W, relief=RIDGE, width=15).grid(row=5, column=0)
        Label(self.frame, text="None", anchor=W, relief=RIDGE, width=15).grid(row=5, column=1)
        Label(self.frame, text="None Scheduled", anchor=W, relief=RIDGE, width=15).grid(row=5, column=2)

        # : Row 1: Friend
        Label(self.frame, text="Friend", anchor=W, relief=RIDGE, width=15).grid(row=6, column=0)
        Label(self.frame, text="None", anchor=W, relief=RIDGE, width=15).grid(row=6, column=1)
        Label(self.frame, text="None Scheduled", anchor=W, relief=RIDGE, width=15).grid(row=6, column=2)

        # : Row 1: Relative
        Label(self.frame, text="Relative", anchor=W, relief=RIDGE, width=15).grid(row=7, column=0)
        Label(self.frame, text="None", anchor=W, relief=RIDGE, width=15).grid(row=7, column=1)
        Label(self.frame, text="None Scheduled", anchor=W, relief=RIDGE, width=15).grid(row=7, column=2)

        # : Row 1: Doctor
        Label(self.frame, text="Doctor", anchor=W, relief=RIDGE, width=15).grid(row=8, column=0)
        Label(self.frame, text="None", anchor=W, relief=RIDGE, width=15).grid(row=8, column=1)
        Label(self.frame, text="None Scheduled", anchor=W, relief=RIDGE, width=15).grid(row=8, column=2)

        # : Row 1: Nurse
        Label(self.frame, text="Nurse", anchor=W, relief=RIDGE, width=15).grid(row=9, column=0)
        Label(self.frame, text="None", anchor=W, relief=RIDGE, width=15).grid(row=9, column=1)
        Label(self.frame, text="None Scheduled", anchor=W, relief=RIDGE, width=15).grid(row=9, column=2)

        # : Row 1: Caregivier
        Label(self.frame, text="Caregivier", anchor=W, relief=RIDGE, width=15).grid(row=10, column=0)
        Label(self.frame, text="None", anchor=W, relief=RIDGE, width=15).grid(row=10, column=1)
        Label(self.frame, text="None Scheduled", anchor=W, relief=RIDGE, width=15).grid(row=10, column=2)

        # create and assign dice label to label widget. Updating dice_label will
        # automatically update the widget's text
        self.dice_label_widget = Label(self.root, textvariable=self.dice_label)
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