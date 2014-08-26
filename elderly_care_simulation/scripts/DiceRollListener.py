#!/usr/bin/env python
import rospy
import roslib
import roslib; roslib.load_manifest('elderly_care_simulation')
from elderly_care_simulation.msg import DiceRollTrigger
from elderly_care_simulation.msg import EventTrigger
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
        "Medication", "Cook", "Entertainment", "Companionship", "Wake", "Sleep"]
        self.eventType = StringVar(self.root)
        self.eventType.set(self.typeOptions[0])

        # Dictionary for mapping event names to their corresponding numbers
        self.typeDict = {"Undefined":0, "Eat":1 , "Shower":2, "Exercise":3, "Conversation":4, "Moral":5, "Relative":6, "Friend":7, "Ill":8,
        "Very Ill":9, "Medication":10, "Cook":11, "Entertainment":12, "Companionship":13, "Wake":14, "Sleep":15}

        # Set up event message menu
        self.messageOptions = ["Undefined", "Request", "Response"]
        self.eventMessage = StringVar(self.root)
        self.eventMessage.set(self.messageOptions[0])

        # Dictionary for mapping message names to their corresponding numbers
        self.messageDict = {"Undefined":0, "Request":1, "Response":2}

        # Set up event priority menu
        self.priorityOptions = ["0", "1", "2", "3", "4", "5", "6"]
        self.eventPriority = StringVar(self.root)
        self.eventPriority.set(self.priorityOptions[0])

        # Set up event weight menu
        self.weightOptions = ["0", "1", "2"]
        self.eventWeight = StringVar(self.root)
        self.eventWeight.set(self.weightOptions[0])

        # Set up result result menu
        self.resultOptions = ["Undefined", "Failure", "Success"]
        self.eventResult = StringVar(self.root)
        self.eventResult.set(self.resultOptions[0])

        # Dictionary for mapping result names to their corresponding numbers
        self.resultDict = {"Undefined":0, "Failure":1, "Success":2}

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

        Button(self.frame_injectEvent, text="Inject", command=self.injectEventCallback).pack(side=TOP, padx=10, pady=5, fill=X)
        
        # Label for event changing
        Label(self.frame_eventChange, text="Change Events", bg='ivory4', width=10).pack(side=TOP, padx=5, pady=5, fill=X)
        Button(self.frame_eventChange, text="Repopulate\nDaily Events", width=10, command=self.repopulateEventsCallback).pack(side=TOP, padx=5, pady=10, fill=X)
        Button(self.frame_eventChange, text="Clear All\nEvents", width=10, command=self.clearEventsCallback).pack(side=TOP, padx=10, pady=10, fill=X)

        # create and assign dice label to label widget. Updating dice_label will
        # automatically update the widget's text
        #self.dice_label_widget = Label(self.frame_events, textvariable=self.dice_label)
        #self.dice_label_widget.pack()

        # initialise listener node
        rospy.init_node('DiceRollListener', anonymous=True)

        # Initialise publisher for injecting events
        self.eventTriggerPub = rospy.Publisher('event_trigger', EventTrigger)

        # subscribe to topic
        rospy.Subscriber("event_trigger", EventTrigger, self.event_trigger_callback)

        # Dictionary which maps different events to different method which will respond to them.
        self.events = {0:self.undefined, 1:self.eat, 2:self.shower, 3:self.exercise, 4:self.converse, 5:self.support, 6:self.relative,
        7:self.friend, 8:self.ill, 9:self.veryIll, 10:self.medication, 11:self.cook, 12:self.entertainment, 13:self.companionship,
        14:self.wake, 15:self.sleep}


    def run(self):
        """
        Initiate tkInter's main loop. This will populate the root window
        with widgets as declared in __init__
        """
        self.root.mainloop()

    def dice_roll_callback(self, msg):
        data = 'Dice Type: %d Threshold: %4d Rolled: %4d' % (msg.type, msg.threshold, msg.rolled)
        # rospy.loginfo("Dice Type: %d Threshold: %4d Rolled: %4d",msg.type, msg.threshold, msg.rolled)
        self.update_dice_label(data)

    # Callback method for event_trigger topic
    def event_trigger_callback(self, msg):
        
        msg_type = msg.msg_type
        event_type = msg.event_type
        result = msg.result

        print("Event trigger detected")
        print("Message type: ", msg_type)
        print("Event type: ", event_type)
        print("result: ", result)

        self.events[msg.event_type](msg)


    # Callback method for repopulating events
    def repopulateEventsCallback(self):
        print("Repopulate!")

    # Callback method for clearing events
    def clearEventsCallback(self):
        print("Clear!")

    # Callback method for clearing events
    def injectEventCallback(self):
        print("INJECT!")
        injectEvent = EventTrigger()
        injectEvent.msg_type = self.messageDict[self.eventMessage.get()]
        injectEvent.event_type = self.typeDict[self.eventType.get()]
        injectEvent.event_priority = int(self.eventPriority.get())
        injectEvent.event_weight = int(self.eventWeight.get())
        injectEvent.result = self.resultDict[self.eventResult.get()]

        self.eventTriggerPub.publish(injectEvent)

    #Update undefined state
    def undefined(self, result):
        return

    #Update eat state
    def eat(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.caregiver_task.set("Feeding")
                self.resident_task.set("Eating")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.caregiver_task.set("None")
                self.resident_task.set("None")

    #Update shower state
    def shower(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.caregiver_task.set("Showering")
                self.resident_task.set("Showering")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.caregiver_task.set("None")
                self.resident_task.set("None")

    #Update exercise state
    def exercise(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.caregiver_task.set("Exercising")
                self.resident_task.set("Exercising")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.caregiver_task.set("None")
                self.resident_task.set("None")

    #Update converse state
    def converse(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.caregiver_task.set("Conversing")
                self.resident_task.set("Conversing")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.caregiver_task.set("None")
                self.resident_task.set("None")

    #Update support state
    def support(self, message):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.caregiver_task.set("Supporting")
                self.resident_task.set("Complaining")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.caregiver_task.set("None")
                self.resident_task.set("None")

    #Update relative state
    def relative(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.relative_task.set("Visiting")
                self.resident_task.set("Interacting")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.relative_task.set("None")
                self.resident_task.set("None")

    #Update friend state
    def friend(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.friend_task.set("Visiting")
                self.resident_task.set("Interacting")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.friend_task.set("None")
                self.resident_task.set("None")

    #Update ill state
    def ill(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.nurse_task.set("Nursing")
                self.resident_task.set("Ill")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.nurse_task.set("None")
                self.resident_task.set("None")

    #Update veryIll state
    def veryIll(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.doctor_task.set("Doctoring")
                self.resident_task.set("Very Ill")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.doctor_task.set("None")
                self.resident_task.set("None")

    #Update medication state
    def medication(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.medication_task.set("Drugging")
                self.resident_task.set("Tripping")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.medication_task.set("None")
                self.resident_task.set("None")

    #Update cook state
    def cook(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.cook_task.set("Cooking")
                self.resident_task.set("Waiting")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.cook_task.set("None")
                self.resident_task.set("None")

    #Update entertainment state
    def entertainment(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.entertainment_task.set("Entertaining")
                self.resident_task.set("Enjoying")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.entertainment_task.set("None")
                self.resident_task.set("None")

    #Update companionship state
    def companionship(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.companion_task.set("Accompanying")
                self.resident_task.set("Talking")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.companion_task.set("None")
                self.resident_task.set("None")

    #Update wake state
    def wake(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.resident_task.set("Waking")
                self.resident_task.set("Waking")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.resident_task.set("None")
                self.resident_task.set("None")

    #Update sleep state
    def sleep(self, result):
        if (message.msg_type == 1):
            if (message.result == 0):
                self.resident_task.set("Sleeping")
                self.resident_task.set("Sleeping")
        elif (message.msg_type == 2):
            if (message.result == 2):
                self.resident_task.set("None")
                self.resident_task.set("None")

    #Update undefined state
    def update_robot_task(self, data):
        self.dice_label.set(data)

    #Update undefined state
    def update_dice_label(self, data):
        self.dice_label.set(data)

if __name__=='__main__':
    gui = DiceRollerGUI()
    gui.run()