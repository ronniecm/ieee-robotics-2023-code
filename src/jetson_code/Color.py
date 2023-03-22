#!/usr/bin/env python3

#==========================================
# Title:  Robot Ranging Class
# Author: Juan Suquilanda
# Date:   27 February 2023
#==========================================

import rospy
from std_msgs.msg import Int8

'''
Class to control servos, by making servo class one will have the ability to talk/
control servos connected to node. We will establish a publisher to controll motion and a 
subcriber to recieve messages from teensy that the action is finished
'''

class Color:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        
        rospy.Subscriber('%s/color' %robot_name, Int8, self.callBack)
        self.isWhite = 1

    def callBack(self, msg):

        if msg.data == 0:
            self.isWhite == 1
        else:
            self.isWhite == 0

class LED:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        rospy.Subscriber('%s/redled' % robot_name, Int8, self.redled)
        self.led = 0


    def redled(self, msg):
        self.led = msg.data

    def getRedLed(self):
        return self.led



    
        

        


