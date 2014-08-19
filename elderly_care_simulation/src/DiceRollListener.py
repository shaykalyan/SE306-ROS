#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('elderly_care_simulation')
from elderly_care_simulation.msg import DiceRollTrigger
from Tkinter import *
import threading
import Queue
from time import sleep
import random

class ROSThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        self.listener()

    def listener(self):

        # initialise listener node
        # anonymous ensures a unique listeners allowing multiple instances
        # of DiceRollListener to running simultanously
        rospy.init_node('DiceRollListener', anonymous=True)

        # subscribe to topic
        rospy.Subscriber("dice_roll", DiceRollTrigger, dice_roll_callback)

        # prevent python from exiting
        rospy.spin()

    def dice_roll_callback(msg):
        rospy.loginfo("Dice Type: %d Threshold: %4d Rolled: %4d",msg.type, msg.threshold, msg.rolled)
        tkThread.thread_0_update(msg.rolled)

class tkinterThread:
    def __init__(self):
        self.master=Tk()
        self.master.geometry('200x200+1+1')

        f=Frame(self.master)
        f.pack()

        self.l0=Label(f)
        self.l0.pack()

        self.q0=Queue.Queue()

        self.master.bind("<<Thread_0_Label_Update>>",self.thread_0_update_e)

    def start(self):
        self.master.mainloop()
        #self.master.destroy()


    def thread_0_update(self,val):
        self.q0.put(val)
        self.master.event_generate('<<Thread_0_Label_Update>>',when='tail')

    def thread_0_update_e(self,e):
        while self.q0.qsize():
            try:
                val=self.q0.get()
                self.l0.config(text=str(val))
            except Queue.Empty:
                pass



if __name__=='__main__':
    tkThread=tkinterThread()
    rThread=ROSThread()
    rThread.start()
    tkThread.start()