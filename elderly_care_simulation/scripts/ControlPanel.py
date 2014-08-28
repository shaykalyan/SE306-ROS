#!/usr/bin/env python
import rospy
import roslib
import rospkg
import roslib; roslib.load_manifest('elderly_care_simulation')
from elderly_care_simulation.msg import DiceRollTrigger, EventTrigger, DiceRollReport, GuiComm
from Tkinter import *
from EventTriggerUtility import *

class DiceRollerGUI:
    """
    TkInter GUI for the visualisation of dice rolls carried out by the 
    various *DiceRoller implementations
    """
    def __init__(self):

        #Initialises tkInter elements and subscribes to relevant ROS Topics

        ######################### SET UP FRAME LAYOUTS #########################

        # create root element with fixed size
        self.root = Tk()
        self.root.wm_title("ROS: Control Panel")
        self.root.geometry('610x590+1+1')

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


        ######################### SET UP ROBOT TASK GRID #########################        

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

        # List for tracking upcoming tasks for the resident
        self.resident_task_list = ["None", "None"]

        # Create constants for grid layout
        gridRelief = RIDGE
        gridAnchor = W
        gridWidth = 15
        gridHeight = 2

        # Creates Labels which will eventually contain information about robots and their tasks
        # :Column Names
        #Label(self.frame_robotGrid, anchor=gridAnchor, relief=gridRelief, bg='ivory4', width=imageWidth).grid(row=0, column=0)
        Label(self.frame_robotGrid, text="Robot Name", anchor=gridAnchor, relief=gridRelief, bg='ivory4', width=gridWidth).grid(row=0, column=1)
        Label(self.frame_robotGrid, text="Current Task", anchor=gridAnchor, relief=gridRelief, bg='ivory4', width=gridWidth).grid(row=0, column=2)

        # : Row 1: Resident
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path("elderly_care_simulation")
        self.resident_image = PhotoImage(file=(self.pkg_path + "/scripts/robot_images/resident.gif"), height=15, width=15)
        Label(self.frame_robotGrid, image=self.resident_image, anchor=gridAnchor, relief=gridRelief, height=15, width=17).grid(row=1, column=0)
        Label(self.frame_robotGrid, text="Resident", anchor=gridAnchor, relief=gridRelief, height=gridHeight, width=gridWidth).grid(row=1, column=1)
        Label(self.frame_robotGrid, textvariable=self.resident_task, anchor=gridAnchor, relief=gridRelief, height=gridHeight, width=gridWidth).grid(row=1, column=2)

        # : Row 2: Cook
        self.cook_image = PhotoImage(file=(self.pkg_path + "/scripts/robot_images/chef.gif"), height=15, width=15)
        Label(self.frame_robotGrid, image=self.cook_image, anchor=gridAnchor, relief=gridRelief, height=15, width=17).grid(row=2, column=0)
        Label(self.frame_robotGrid, text="Cook", anchor=gridAnchor, relief=gridRelief, height=gridHeight, width=gridWidth).grid(row=2, column=1)
        Label(self.frame_robotGrid, textvariable=self.cook_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=2, column=2)

        # : Row 3: Medication
        self.medication_image = PhotoImage(file=(self.pkg_path + "/scripts/robot_images/medication.gif"), height=15, width=15)
        Label(self.frame_robotGrid, image=self.medication_image, anchor=gridAnchor, relief=gridRelief, height=15, width=15).grid(row=3, column=0)
        Label(self.frame_robotGrid, text="Medication", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=3, column=1)
        Label(self.frame_robotGrid, textvariable=self.medication_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=3, column=2)

        # : Row 4: Entertainment
        self.entertainment_image = PhotoImage(file=(self.pkg_path + "/scripts/robot_images/entertainment.gif"), height=15, width=15)
        Label(self.frame_robotGrid, image=self.entertainment_image, anchor=gridAnchor, relief=gridRelief, height=15, width=15).grid(row=4, column=0)
        Label(self.frame_robotGrid, text="Entertainment", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=4, column=1)
        Label(self.frame_robotGrid, textvariable=self.entertainment_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=4, column=2)

        # : Row 5: Companionship
        self.companionship_image = PhotoImage(file=(self.pkg_path + "/scripts/robot_images/companionship.gif"), height=15, width=15)
        Label(self.frame_robotGrid, image=self.companionship_image, anchor=gridAnchor, relief=gridRelief, height=15, width=15).grid(row=5, column=0)
        Label(self.frame_robotGrid, text="Companionship", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=5, column=1)
        Label(self.frame_robotGrid, textvariable=self.companion_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=5, column=2)

        # : Row 6: Friend
        self.friend_image = PhotoImage(file=(self.pkg_path + "/scripts/robot_images/friend.gif"), height=15, width=15)
        Label(self.frame_robotGrid, image=self.friend_image, anchor=gridAnchor, relief=gridRelief, height=15, width=15).grid(row=6, column=0)
        Label(self.frame_robotGrid, text="Friend", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=6, column=1)
        Label(self.frame_robotGrid, textvariable=self.friend_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=6, column=2)

        # : Row 7: Relative
        self.relative_image = PhotoImage(file=(self.pkg_path + "/scripts/robot_images/relative.gif"), height=15, width=15)
        Label(self.frame_robotGrid, image=self.relative_image, anchor=gridAnchor, relief=gridRelief, height=15, width=15).grid(row=7, column=0)
        Label(self.frame_robotGrid, text="Relative", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=7, column=1)
        Label(self.frame_robotGrid, textvariable=self.relative_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=7, column=2)

        # : Row 8: Doctor
        self.doctor_image = PhotoImage(file=(self.pkg_path + "/scripts/robot_images/doctor.gif"), height=15, width=15)
        Label(self.frame_robotGrid, image=self.doctor_image, anchor=gridAnchor, relief=gridRelief, height=15, width=15).grid(row=8, column=0)
        Label(self.frame_robotGrid, text="Doctor", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=8, column=1)
        Label(self.frame_robotGrid, textvariable=self.doctor_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=8, column=2)

        # : Row 9: Nurse
        self.nurse_image = PhotoImage(file=(self.pkg_path + "/scripts/robot_images/nurse.gif"), height=15, width=15)
        Label(self.frame_robotGrid, image=self.nurse_image, anchor=gridAnchor, relief=gridRelief, height=15, width=15).grid(row=9, column=0)
        Label(self.frame_robotGrid, text="Nurse", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=9, column=1)
        Label(self.frame_robotGrid, textvariable=self.nurse_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=9, column=2)

        # : Row 10: Caregivier
        self.caregiver_image = PhotoImage(file=(self.pkg_path + "/scripts/robot_images/caregiver.gif"), height=15, width=15)
        Label(self.frame_robotGrid, image=self.caregiver_image, anchor=gridAnchor, relief=gridRelief, height=15, width=15).grid(row=10, column=0)
        Label(self.frame_robotGrid, text="Caregivier", anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=10, column=1)
        Label(self.frame_robotGrid, textvariable=self.caregiver_task, anchor=gridAnchor, relief=gridRelief, width=gridWidth, height=gridHeight).grid(row=10, column=2)

        # Dictionary which maps different events to different method which will respond to them.
        self.events = {ET_EVENT_TYPE_UNDEFINED : self.undefined, ET_EVENT_TYPE_EAT : self.eat, ET_EVENT_TYPE_SHOWER : self.shower,
        ET_EVENT_TYPE_EXERCISE : self.exercise, ET_EVENT_TYPE_CONVERSATION : self.converse, ET_EVENT_TYPE_MORAL_SUPPORT : self.support,
        ET_EVENT_TYPE_RELATIVE : self.relative, ET_EVENT_TYPE_FRIEND : self.friend, ET_EVENT_TYPE_ILL : self.ill,
        ET_EVENT_TYPE_VERY_ILL : self.veryIll, ET_EVENT_TYPE_MEDICATION : self.medication, ET_EVENT_TYPE_COOK : self.cook,
        ET_EVENT_TYPE_ENTERTAINMENT : self.entertainment, ET_EVENT_TYPE_COMPANIONSHIP : self.companionship, ET_EVENT_TYPE_WAKE : self.wake,
        ET_EVENT_TYPE_SLEEP : self.sleep, ET_EVENT_TYPE_MOVE_TO_KITCHEN : self.move_kitchen, ET_EVENT_TYPE_MOVE_TO_BEDROOM : self.move_bedroom,
        ET_EVENT_TYPE_MOVE_TO_HALLWAY : self.move_hallway, ET_EVENT_TYPE_MOVE_TO_TOILET : self.move_toilet}

        
        ######################### SET UP CURRENT EVENT LIST #########################

        # Create variable for current and upcomming events
        self.current_event_1 = StringVar()
        self.current_event_1.set("")
        self.current_event_2 = StringVar()
        self.current_event_2.set("")
        self.current_event_3 = StringVar()
        self.current_event_3.set("")
        self.next_event = StringVar()
        self.next_event.set("")

        # Create a list for monitoring the current tasks
        self.current_events = [self.current_event_1, self.current_event_2, self.current_event_3]

        # Set the Labels that contain information about current events
        Label(self.frame_events, text="Current Events", relief=gridRelief, bg='ivory4').pack(fill=X)
        Label(self.frame_events, textvariable=self.current_event_1, relief=gridRelief).pack(fill=X)
        Label(self.frame_events, textvariable=self.current_event_2, relief=gridRelief).pack(fill=X)
        Label(self.frame_events, textvariable=self.current_event_3, relief=gridRelief).pack(fill=X)
        Label(self.frame_events, text="").pack(fill=X)
        Label(self.frame_events, text="Upcoming Event", relief=gridRelief, bg='ivory4').pack(fill=X)
        Label(self.frame_events, textvariable=self.next_event, relief=gridRelief).pack(fill=X)


        ######################### SET UP DICE ROLLER MONITOR #########################

        # Label for the dice rollers
        Label(self.frame_diceRolls, text="Dice Rollers", relief=gridRelief, bg='ivory4', width=68).pack(side=TOP)

        # Create ill dice roller label
        self.ill_label = Canvas(bg='gray70', width=550, height=15)
        self.ill_label.pack(side=TOP)
        self.ill_label.create_text(275, 8, text="Ill Dice Roller")

        # Set up ill dice roll canvas
        #Label(self.frame_diceRolls, text="Ill Dice Roller", relief=gridRelief, bg='ivory2', width=68).pack(side=TOP)
        self.ill_canvas = Canvas(bg='snow', width=550, height=30)
        self.ill_canvas.pack(side=TOP)
        self.ill_rectangle = self.ill_canvas.create_rectangle(0, 0, 500, 30, fill='PaleGreen4')
        self.ill_roll = self.ill_canvas.create_rectangle(0, 0, 2, 30, fill='red')

        # Create very ill dice roller label
        self.very_ill_label = Canvas(bg='gray70', width=550, height=15)
        self.very_ill_label.pack(side=TOP)
        self.very_ill_label.create_text(275, 8, text="Very Ill Dice Roller")

        # Set up very ill dice roll canvas
        #Label(self.frame_diceRolls, text="Very Ill Dice Roller", relief=gridRelief, bg='ivory2', width=68).pack(side=TOP)
        self.very_ill_canvas = Canvas(bg='snow', width=550, height=30)
        self.very_ill_canvas.pack(side=TOP)
        self.very_ill_rectangle = self.very_ill_canvas.create_rectangle(0, 0, 500, 30, fill='PaleGreen4')
        self.very_ill_roll = self.very_ill_canvas.create_rectangle(0, 0, 2, 30, fill='red')

        # Create moral support dice roller label
        self.moral_support = Canvas(bg='gray70', width=550, height=15)
        self.moral_support.pack(side=TOP)
        self.moral_support.create_text(275, 8, text="Moral Support Dice Roller")

        # Set up very ill dice roll canvas
        #Label(self.frame_diceRolls, text="Moral Support Dice Roller", relief=gridRelief, bg='ivory2', width=68).pack(side=TOP)
        self.moral_canvas = Canvas(bg='snow', width=550, height=30)
        self.moral_canvas.pack(side=TOP)
        self.moral_rectangle = self.moral_canvas.create_rectangle(0, 0, 500, 30, fill='PaleGreen4')
        self.moral_roll = self.moral_canvas.create_rectangle(0, 0, 2, 30, fill='red')

        
        ######################### SET UP EVENT INJECTION #########################

        # Set up event type dropdown menu
        self.typeOptions = ["Undefined", "Shower", "Exercise", "Conversation", "Moral", "Relative", "Friend", "Ill", "Very Ill", 
        "Medication", "Cook", "Entertainment", "Companionship", "Wake", "Sleep", "Kitchen", "Bedroom", "Hallway", "Toilet"]
        self.eventType = StringVar(self.root)
        self.eventType.set(self.typeOptions[0])

        # Dictionary for mapping event names to their corresponding numbers
        self.typeDict = {"Undefined" : ET_EVENT_TYPE_UNDEFINED, "Shower" : ET_EVENT_TYPE_SHOWER,
        "Exercise" : ET_EVENT_TYPE_EXERCISE, "Conversation" : ET_EVENT_TYPE_CONVERSATION, "Moral" : ET_EVENT_TYPE_MORAL_SUPPORT,
        "Relative" : ET_EVENT_TYPE_RELATIVE, "Friend" : ET_EVENT_TYPE_FRIEND, "Ill" : ET_EVENT_TYPE_ILL, "Very Ill" : ET_EVENT_TYPE_VERY_ILL,
        "Medication" : ET_EVENT_TYPE_MEDICATION, "Cook" : ET_EVENT_TYPE_COOK, "Entertainment" : ET_EVENT_TYPE_ENTERTAINMENT,
        "Companionship" : ET_EVENT_TYPE_COMPANIONSHIP, "Wake" : ET_EVENT_TYPE_WAKE, "Sleep" : ET_EVENT_TYPE_SLEEP, "Kitchen" : ET_EVENT_TYPE_MOVE_TO_KITCHEN,
         "Bedroom" : ET_EVENT_TYPE_MOVE_TO_BEDROOM, "Hallway" : ET_EVENT_TYPE_MOVE_TO_HALLWAY, "Toilet" : ET_EVENT_TYPE_MOVE_TO_TOILET}

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

        Label(self.frame_injectEvent, text="Event Result", bg='ivory2', width=15).pack(side=TOP, padx=10, fill=X)
        OptionMenu(self.frame_injectEvent, self.eventResult, *self.resultOptions).pack(side=TOP, padx=10, pady=5, fill=X)

        Button(self.frame_injectEvent, text="Inject", command=self.injectEventCallback).pack(side=TOP, padx=10, pady=5, fill=X)
        
        
        ######################### SET UP EVENT MANIPULATION #########################

        # Label for event changing
        Label(self.frame_eventChange, text="Change Events", bg='ivory4', width=10).pack(side=TOP, padx=5, pady=5, fill=X)
        Button(self.frame_eventChange, text="Repopulate\nDaily Events", width=10, command=self.repopulateEventsCallback).pack(side=TOP, padx=10, pady=10, fill=X)
        Button(self.frame_eventChange, text="Clear All\nEvents", width=10, command=self.clearEventsCallback).pack(side=TOP, padx=10, pady=10, fill=X)


        ######################### SET UP ROSPY #########################

        # initialise listener node
        rospy.init_node('DiceRollListener', anonymous=True)

        # Initialise publisher for injecting events

        self.eventTriggerPub = rospy.Publisher('event_trigger', EventTrigger, queue_size=1000)

        # Initialise publisher for clearing or populating the schedule
        self.guiCommunication = rospy.Publisher('gui_communication', GuiComm, queue_size=1000)

        # subscribe to EventTrigger topic
        rospy.Subscriber("event_trigger", EventTrigger, self.event_trigger_callback)

        # subscribe to DiceRollReport topic
        rospy.Subscriber("dice_roll_report", DiceRollReport, self.roll_report_callback)

        # subscribe to GuiComm topic
        rospy.Subscriber("gui_communication", GuiComm, self.upcomming_event_callback)


    def run(self):

        """Initiate tkInter's main loop. This will populate the root window
        with widgets as declared in __init__"""
        self.root.mainloop()

    def roll_report_callback(self, report):
        if (report.type == 0):
            barWidth = int((float(report.threshold)/float(report.numSides))*550)
            rollPoint = int((float(report.rolled)/float(report.numSides))*550) - 1
            self.moral_canvas.coords(self.very_ill_rectangle, 0, 0, barWidth, 30)
            self.moral_canvas.coords(self.very_ill_roll, rollPoint, 0, (rollPoint+2), 30)
        elif (report.type == 1):
            barWidth = int((float(report.threshold)/float(report.numSides))*550)
            rollPoint = int((float(report.rolled)/float(report.numSides))*550) - 1
            self.ill_canvas.coords(self.ill_rectangle, 0, 0, barWidth, 30)
            self.ill_canvas.coords(self.ill_roll, rollPoint, 0, (rollPoint+2), 30)
        elif (report.type == 2):
            barWidth = int((float(report.threshold)/float(report.numSides))*550)
            rollPoint = int((float(report.rolled)/float(report.numSides))*550) - 1
            self.very_ill_canvas.coords(self.very_ill_rectangle, 0, 0, barWidth, 30)
            self.very_ill_canvas.coords(self.very_ill_roll, rollPoint, 0, (rollPoint+2), 30)

    # Callback method for event_trigger topic
    def event_trigger_callback(self, msg):
        self.events[msg.event_type](msg)


    # Callback method for repopulating events
    def repopulateEventsCallback(self):
        guiMessage = GuiComm()
        guiMessage.msg_type = 1
        guiMessage.action = 1
        guiMessage.event_type = 0

        self.guiCommunication.publish(guiMessage)

    # Callback method for clearing events
    def clearEventsCallback(self):
        guiMessage = GuiComm()
        guiMessage.msg_type = 1
        guiMessage.action = 2
        guiMessage.event_type = 0

        self.guiCommunication.publish(guiMessage)

    # Method for updating the upcoming event
    def upcomming_event_callback(self, message):
        if (message.event_type == 0):
            self.next_event.set("")
        else:
            self.next_event.set(typeToStringDict[message.event_type])
            rospy.loginfo("Event queued" + typeToStringDict[message.event_type])

    # Callback method for clearing events
    def injectEventCallback(self):

        # Retrieve injected values and put them in a message
        injectEvent = EventTrigger()
        injectEvent.msg_type = self.messageDict[self.eventMessage.get()]
        injectEvent.event_type = self.typeDict[self.eventType.get()]
        injectEvent.event_priority = int(self.eventPriority.get())
        injectEvent.event_weight = getEventWeight(injectEvent.event_type)
        injectEvent.result = self.resultDict[self.eventResult.get()]

        self.eventTriggerPub.publish(injectEvent)

        # If sleep is 'injected', follow it up with a 'wake' event
        if (injectEvent.event_type == ET_EVENT_TYPE_SLEEP):
            injectEvent = EventTrigger()
            injectEvent.msg_type = ET_MSG_TYPE_REQUEST
            injectEvent.event_type = ET_EVENT_TYPE_WAKE
            injectEvent.event_priority = ET_EVENT_PRIORITY_MEDIUM
            injectEvent.event_weight = getEventWeight(ET_EVENT_TYPE_WAKE)
            injectEvent.result = ET_EVENT_RESULT_UNDEFINED
            self.eventTriggerPub.publish(injectEvent)

    #Update undefined state
    def undefined(self, result):
        return

    #Update eat state
    def eat(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.caregiver_task.set("Feeding")
                self.new_resident_task("Eating")
                self.addCurrentEvents("Eating")
                if (self.next_event.get() == "Eating"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.caregiver_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Eating")

    #Update shower state
    def shower(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.caregiver_task.set("Showering")
                self.new_resident_task("Showering")
                self.addCurrentEvents("Showering")
                if (self.next_event.get() == "Showering"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.caregiver_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Showering")

    #Update exercise state
    def exercise(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.caregiver_task.set("Exercising")
                self.new_resident_task("Exercising")
                self.addCurrentEvents("Exercising")
                if (self.next_event.get() == "Exercising"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.caregiver_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Exercising")

    #Update converse state
    def converse(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.caregiver_task.set("Conversing")
                self.new_resident_task("Conversing")
                self.addCurrentEvents("Conversation")
                if (self.next_event.get() == "Conversation"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.caregiver_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Conversation")

    #Update support state
    def support(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.caregiver_task.set("Supporting")
                self.new_resident_task("Complaining")
                self.addCurrentEvents("Moral")
                if (self.next_event.get() == "Moral"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.caregiver_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Moral Support")

    #Update relative state
    def relative(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.relative_task.set("Visiting")
                self.new_resident_task("Interacting")
                self.addCurrentEvents("Relative")
                if (self.next_event.get() == "Relative"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.relative_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Relative")

    #Update friend state
    def friend(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.friend_task.set("Visiting")
                self.new_resident_task("Interacting")
                self.addCurrentEvents("Friend")
                if (self.next_event.get() == "Friend"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.friend_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Friend")

    #Update ill state
    def ill(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.nurse_task.set("Nursing")
                self.new_resident_task("Ill")
                self.addCurrentEvents("Ill")
                if (self.next_event.get() == "Ill"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.nurse_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Ill")

    #Update veryIll state
    def veryIll(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.doctor_task.set("Doctoring")
                self.new_resident_task("Very Ill")
                self.addCurrentEvents("Very Ill")
                if (self.next_event.get() == "Very Ill"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.doctor_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Very Ill")

    #Update medication state
    def medication(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.medication_task.set("Drugging")
                self.new_resident_task("Medicating")
                self.addCurrentEvents("Medication")
                if (self.next_event.get() == "Medication"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.medication_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Medication")

    #Update cook state
    def cook(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.cook_task.set("Cooking")
                self.new_resident_task("Waiting")
                self.addCurrentEvents("Cooking")
                if (self.next_event.get() == "Cooking"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.cook_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Cooking")

    #Update entertainment state
    def entertainment(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.entertainment_task.set("Entertaining")
                self.new_resident_task("Enjoying")
                self.addCurrentEvents("Entertainment")
                if (self.next_event.get() == "Entertainment"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.entertainment_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Entertainment")

    #Update companionship state
    def companionship(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.companion_task.set("Accompanying")
                self.new_resident_task("Talking")
                self.addCurrentEvents("Companionship")
                if (self.next_event.get() == "Companionship"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.companion_task.set("None")
                self.resident_task_done()
                self.removeCurrentEvents("Companionship")

    #Update wake state
    def wake(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.new_resident_task("Waking")
                if (self.next_event.get() == "Waking"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.resident_task_done()

    #Update sleep state
    def sleep(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.new_resident_task("Sleeping")
                if (self.next_event.get() == "Sleeping"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.resident_task_done()

    #Update sleep state
    def move_kitchen(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.new_resident_task("Walk kitchen")
                self.addCurrentEvents("Walk kitchen")
                if (self.next_event.get() == "Walk kitchen"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.resident_task_done()
                self.removeCurrentEvents("Walk kitchen")

    #Update sleep state
    def move_bedroom(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.new_resident_task("Walk bedroom")
                self.addCurrentEvents("Walk bedroom")
                if (self.next_event.get() == "Walk bedroom"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.resident_task_done()
                self.removeCurrentEvents("Walk bedroom")

    #Update sleep state
    def move_hallway(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.new_resident_task("Walk hallway")
                self.addCurrentEvents("Walk hallway")
                if (self.next_event.get() == "Walk hallway"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.resident_task_done()
                self.removeCurrentEvents("Walk hallway")

    #Update sleep state
    def move_toilet(self, message):
        if (message.msg_type == ET_MSG_TYPE_REQUEST):
            if (message.result == ET_EVENT_RESULT_UNDEFINED):
                self.new_resident_task("Walk toilet")
                self.addCurrentEvents("Walk toilet")
                if (self.next_event.get() == "Walk toilet"):
                    self.next_event.set("")
        elif (message.msg_type == ET_MSG_TYPE_RESPONSE):
            if (message.result == ET_EVENT_RESULT_SUCCESS):
                self.resident_task_done()
                self.removeCurrentEvents("Walk toilet")

    # Update task status for resident with new task
    def new_resident_task(self, task):
        if (self.resident_task_list[0] == "None"):
            self.resident_task_list[0] = task
            self.resident_task.set(task)
        elif (self.resident_task_list[1] == "None"):
            self.resident_task_list[1] = task
        else:
            rospy.loginfo("Control Panel: Too many tasks for the sesident " + task)

    # Update task status for resident by removing a completed task 
    def resident_task_done(self):
        if (self.resident_task_list[1] == "None"):
            self.resident_task_list[0] = "None"
            self.resident_task.set("None")
        else:
            self.resident_task_list[0] = self.resident_task_list[1]
            self.resident_task_list[1] = "None"
            self.resident_task.set(self.resident_task_list[0])

    # Method for adding a currently occuring event to the current event list
    def addCurrentEvents(self, newEvent):
        for current in self.current_events:
            if (current.get() == ""):
                current.set(newEvent)
                break
            elif (current.get() == newEvent):
                break

    # Method for removing a finished occuring event from the current event list
    def removeCurrentEvents(self, currentEvent):
        for current in self.current_events:
            if (current.get() == currentEvent):
                current.set("")

if __name__=='__main__':
    gui = DiceRollerGUI()
    gui.run()