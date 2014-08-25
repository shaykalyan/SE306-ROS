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

        #Initialises tkInter elements and subscribes to relevant ROS Topics

        # create root element with fixed size
        self.root = Tk()
        self.root.geometry('570x535+1+1')

        # create the left most frame which will hold an embeded frame with the grid of current tasks being performed ny the robots
        self.frame_top = Frame(self.root)
        self.frame_top.pack(side=TOP, fill=BOTH)

        # create the left most frame which will hold an embeded frame with the grid of current tasks being performed ny the robots
        self.frame_bottom = Frame(self.root)
        self.frame_bottom.pack(side=TOP, fill=BOTH)

        # create the left most frame which will hold an embeded frame with the grid of current tasks being performed ny the robots
        self.frame_left = Frame(self.frame_top)
        self.frame_left.pack(side=LEFT, fill=BOTH)

        # create the left most frame which will hold an embeded frame with the grid of current tasks being performed ny the robots
        self.frame_middle = Frame(self.frame_top)
        self.frame_middle.pack(side=LEFT, fill=BOTH)

        # create right most frame which will hold other embeded frames detailing dice rolls, upcoming events and allow event injection
        self.frame_right = Frame(self.frame_top)        
        self.frame_right.pack(side=LEFT, fill=BOTH)

        # create frame element to host the grid detailing current tasks being performed by robots
        self.frame_robotGrid = Frame(self.frame_left, bd=3)        
        self.frame_robotGrid.pack(side=TOP, padx=10, pady=20)

        # create frame element to host upcoming events and dice rolls
        self.frame_events = Frame(self.frame_middle)        
        self.frame_events.pack(side=TOP, padx=10, pady=20)

        # create frame element to host the event injection interface
        self.frame_eventChange = Frame(self.frame_middle)        
        self.frame_eventChange.pack(side=TOP, padx=10, pady=10, fill=BOTH)

        # create frame element to host the event injection interface
        self.frame_injectEvent = Frame(self.frame_right)        
        self.frame_injectEvent.pack(side=TOP, padx=10, pady=20, fill=BOTH)

        # create frame element to host the event injection interface
        self.frame_diceRolls = Frame(self.frame_bottom)        
        self.frame_diceRolls.pack(side=TOP, padx=10, fill=BOTH)
        

        # create task variable labels
        self.resident_task = StringVar()
        self.resident_task.set("None")
        self.cook_task = StringVar()
        self.cook_task.set("None")
        self.medication_task = StringVar()
        self.medication_task.set("None")
        self.entertainment_task = StringVar()
        self.entertainment_task.set("None")
        self.companion_task = StringVar()
        self.companion_task.set("None")
        self.friend_task = StringVar()
        self.friend_task.set("None")
        self.relative_task = StringVar()
        self.relative_task.set("None")
        self.doctor_task = StringVar()
        self.doctor_task.set("None")
        self.nurse_task = StringVar()
        self.nurse_task.set("None")
        self.caregiver_task = StringVar()
        self.caregiver_task.set("None")


        self.dice_label = StringVar()

        gridRelief = RIDGE
        gridAnchor = W
        gridWidth = 15
        gridHeight = 2

        # Creates Labels which will eventually contain information about robots and their tasks
        # :Column Names
        Label(self.frame_robotGrid, text="Robot Name", anchor=gridAnchor, relief=gridRelief, bg='ivory4', width=gridWidth).grid(row=0, column=0)
        Label(self.frame_robotGrid, text="Current Task", anchor=gridAnchor, relief=gridRelief, bg='ivory4', width=gridWidth).grid(row=0, column=1)

        # : Row 1: Resident
        Label(self.frame_robotGrid, text="Resident", anchor=gridAnchor, relief=gridRelief, height=gridHeight, width=gridWidth).grid(row=1, column=0)
        Label(self.frame_robotGrid, textvariable=self.resident_task, anchor=gridAnchor, relief=gridRelief, height=gridHeight, width=gridWidth).grid(row=1, column=1)

        # : Row 1: Cook
        Label(self.frame_robotGrid, text="Cook", anchor=gridAnchor, relief=gridRelief, height=gridHeight, width=gridWidth).grid(row=2, column=0)
        Label(self.frame_robotGrid, textvariable=self.cook_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=2, column=1)

        # : Row 1: Medication
        Label(self.frame_robotGrid, text="Medication", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=3, column=0)
        Label(self.frame_robotGrid, textvariable=self.medication_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=3, column=1)

        # : Row 1: Entertainment
        Label(self.frame_robotGrid, text="Entertainment", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=4, column=0)
        Label(self.frame_robotGrid, textvariable=self.entertainment_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=4, column=1)

        # : Row 1: Companionship
        Label(self.frame_robotGrid, text="Companionship", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=5, column=0)
        Label(self.frame_robotGrid, textvariable=self.companion_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=5, column=1)

        # : Row 1: Friend
        Label(self.frame_robotGrid, text="Friend", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=6, column=0)
        Label(self.frame_robotGrid, textvariable=self.friend_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=6, column=1)

        # : Row 1: Relative
        Label(self.frame_robotGrid, text="Relative", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=7, column=0)
        Label(self.frame_robotGrid, textvariable=self.relative_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=7, column=1)

        # : Row 1: Doctor
        Label(self.frame_robotGrid, text="Doctor", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=8, column=0)
        Label(self.frame_robotGrid, textvariable=self.doctor_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=8, column=1)

        # : Row 1: Nurse
        Label(self.frame_robotGrid, text="Nurse", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=9, column=0)
        Label(self.frame_robotGrid, textvariable=self.nurse_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=9, column=1)

        # : Row 1: Caregivier
        Label(self.frame_robotGrid, text="Caregivier", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=10, column=0)
        Label(self.frame_robotGrid, textvariable=self.caregiver_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=10, column=1)

        # Set the Labels that contain information about current events
        Label(self.frame_events, text="Current Events", relief=gridRelief, bg='ivory4').pack(fill=X)
        Label(self.frame_events, text="Cooking", relief=gridRelief).pack(fill=X)
        Label(self.frame_events, text="Medication", relief=gridRelief).pack(fill=X)
        Label(self.frame_events, text="", relief=gridRelief).pack(fill=X)
        Label(self.frame_events, text="").pack(fill=X)
        Label(self.frame_events, text="Upcoming Events", relief=gridRelief, bg='ivory4').pack(fill=X)
        Label(self.frame_events, text="Shower", relief=gridRelief).pack(fill=X)

        # Label for the dice rollers
        Label(self.frame_diceRolls, text="Dice Rollers", relief=gridRelief, bg='ivory4', width=40).pack(side=TOP)
        Label(self.frame_diceRolls, text="Ill", relief=gridRelief, bg='SpringGreen4', width=40, height=2).pack(side=TOP)
        Label(self.frame_diceRolls, text="Very Ill", relief=gridRelief, bg='SpringGreen4', width=40, height=2).pack(side=TOP)
        Label(self.frame_diceRolls, text="Moral Support", relief=gridRelief, bg='SpringGreen4', width=40, height=2).pack(side=TOP)

        # Set up event type dropdown menu
        self.typeOptions = ["Undefined", "Eat", "Shower", "Exercise", "Conversation", "Moral", "Relative", "Friend", "Ill", "Very Ill", 
        "Medication", "Cook", "Entertainment", "Companionship"]
        self.eventType = StringVar(self.root)
        self.eventType.set(self.typeOptions[0])

        # Set up event message menu
        self.messageOptions = ["Undefined", "Request", "Response"]
        self.eventMessage = StringVar(self.root)
        self.eventMessage.set(self.messageOptions[0])

        # Set up event priority menu
        self.priorityOptions = ["0", "1", "2", "3", "4", "5", "6"]
        self.eventPriority = StringVar(self.root)
        self.eventPriority.set(self.priorityOptions[0])

        # Set up event weight menu
        self.weightOptions = ["0", "1", "2"]
        self.eventWeight = StringVar(self.root)
        self.eventWeight.set(self.weightOptions[0])

        # Set up result result menu
        self.resultOptions = ["Undefined", "Success", "Failure"]
        self.eventResult = StringVar(self.root)
        self.eventResult.set(self.resultOptions[0])

        # Create dropdown menus and inject button
        # Label for event injection
        Label(self.frame_injectEvent, text="Event Injection", bg='ivory4', width=15).pack(side=TOP, padx=10, pady=5, fill=X)

        Label(self.frame_injectEvent, text="Event Type", bg='ivory2', width=15).pack(side=TOP, padx=10, fill=X)
        OptionMenu(self.frame_injectEvent, self.eventType, *self.typeOptions).pack(side=TOP, padx=10, pady=5, fill=X)

        Label(self.frame_injectEvent, text="Event Message", bg='ivory2', width=15).pack(side=TOP, padx=10, fill=X)
        OptionMenu(self.frame_injectEvent, self.eventMessage, *self.messageOptions).pack(side=TOP, padx=10, pady=5, fill=X)

        Label(self.frame_injectEvent, text="Event Priority", bg='ivory2', width=15).pack(side=TOP, padx=10, fill=X)
        OptionMenu(self.frame_injectEvent, self.eventPriority, *self.priorityOptions).pack(side=TOP, padx=10, pady=5, fill=X)

        Label(self.frame_injectEvent, text="Event Weight", bg='ivory2', width=15).pack(side=TOP, padx=10, fill=X)
        OptionMenu(self.frame_injectEvent, self.eventWeight, *self.weightOptions).pack(side=TOP, padx=10, pady=5, fill=X)

        Label(self.frame_injectEvent, text="Event Result", bg='ivory2', width=15).pack(side=TOP, padx=10, fill=X)
        OptionMenu(self.frame_injectEvent, self.eventResult, *self.resultOptions).pack(side=TOP, padx=10, pady=5, fill=X)

        Button(self.frame_injectEvent, text="Inject").pack(side=TOP, padx=10, pady=5, fill=X)
        
        # Label for event changing
        Label(self.frame_eventChange, text="Change Events", bg='ivory4', width=10).pack(side=TOP, padx=5, pady=5, fill=X)
        Button(self.frame_eventChange, text="Repopulate\nDaily Events", width=10).pack(side=TOP, padx=5, pady=10, fill=X)
        Button(self.frame_eventChange, text="Clear All\nEvents").pack(side=TOP, padx=10, pady=10, fill=X)

        # create and assign dice label to label widget. Updating dice_label will
        # automatically update the widget's text
        #self.dice_label_widget = Label(self.frame_events, textvariable=self.dice_label)
        #self.dice_label_widget.pack()

        # initialise listener node
        # anonymous ensures a unique listeners allowing multiple instances
        # of DiceRollListener to running simultanously
        rospy.init_node('DiceRollListener', anonymous=True)

        # subscribe to topic
        rospy.Subscriber("event_trigger", EventTrigger, self.event_trigger_callback)

        self.events = {0:undefined, 1:eat, 2:shower, 3:exercise, 4:conversation, 5:support, 6:relative, 7:friend, 8:ill, 9:veryIll,
        10:medication, 11:cook, 12:entertainment, 13:companionship, 14:wake, 15:sleep}


    def run(self):
        """
        Initiate tkInter's main loop. This will populate the root window
        with widgets as declared in __init__
        """
        self.root.mainloop()

    def event_trigger_callback(self, msg):
        msg_type = msg.msg_type
        event_type = msg.event_type
        event_priority = msg.event_priority
        event_weight = msg.event_weight
        result = msg.result

        if (msg_type == 0):
            self.events[event_type](result)


    def undefined(self, result):
        if (result == 0):
            self.resident_task.set("None")

    def eat(self, result):
        if (result == 0):
            self.caregiver_task.set("Feeding")

    def shower(self, result):
        if (result == 0):
            self.caregiver_task.set("Showering")

    def exercise(self, result):
        if (result == 0):
            self.caregiver_task.set("Exercising")

    def converse(self, result):
        if (result == 0):
            self.caregiver_task.set("Conversing")

    def support(self, result):
        if (result == 0):
            self.caregiver_task.set("Supporting")

    def relative(self, result):
        if (result == 0):
            self.relative_task.set("Visiting")

    def friend(self, result):
        if (result == 0):
            self.friend_task.set("Visiting")

    def ill(self, result):
        if (result == 0):
            self.nurse_task.set("Nursing")

    def veryIll(self, result):
        if (result == 0):
            self.doctor_task.set("Doctoring")

    def medication(self, result):
        if (result == 0):
            self.medication_task.set("Drugging")

    def cook(self, result):
        if (result == 0):
            self.cook_task.set("Cooking")

    def entertainment(self, result):
        if (result == 0):
            self.entertainment_task.set("Entertaining")

    def companionship(self, result):
        if (result == 0):
            self.companion_task.set("Accompanying")

    def wake(self, result):
        if (result == 0):
            self.resident_task.set("Waking")

    def sleep(self, result):
        if (result == 0):
            self.resident_task.set("Sleeping")

    def update_robot_task(self, data):
        self.dice_label.set(data)

    def update_dice_label(self, data):
        self.dice_label.set(data)

    def dice_roll_callback(self, msg):
        data = 'Dice Type: %d Threshold: %4d Rolled: %4d' % (msg.type, msg.threshold, msg.rolled)
        # rospy.loginfo("Dice Type: %d Threshold: %4d Rolled: %4d",msg.type, msg.threshold, msg.rolled)
        self.update_dice_label(data)

if __name__=='__main__':
    gui = DiceRollerGUI()
    gui.run()