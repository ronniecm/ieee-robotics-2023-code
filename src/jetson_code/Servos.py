#!/usr/bin/env python3

#==========================================
# Title:  Robot Ranging Class
# Author: Juan Suquilanda
# Date:   27 February 2023
#==========================================

import rospy
from std_msgs.msg import Int16

'''
Class to control servos, by making servo class one will have the ability to talk/
control servos connected to node. We will establish a publisher to controll motion and a 
subcriber to recieve messages from teensy that the action is finished
'''

class Servos:
    def __init__(self, robot_name, node_name, command_topic, queue_size):
        self.robot_name = robot_name
        
        self.pub = rospy.Publisher("%s/%s_cmd" %(robot_name,command_topic), Int16, queue_size =10)
        # This is for the subscriber
        
        rospy.Subscriber('%s/%s_%s' %(robot_name, command_topic,"callback"), Int16, self.callBack)

        self.actionComplete = False

        self.servoCmds = dict()

        #Going to make a dictionary that holds servo Starting and End Position
        self.servoCmds['gripperRotateDefault'] = 90
        self.servoCmds['gripperRotate90'] = 0
        self.servoCmds['gripperClampOpen'] = 180
        self.servoCmds['gripperClampClosed'] = 0
        self.servoCmds['wristDefault'] = 180
        self.servoCmds['wristAdjust'] = 180
        self.servoCmds['armDown'] = 180
        self.servoCmds['armUp'] = 0
        self.servoCmds['paddleClosed'] = 180
        self.servoCmds['paddleOpen'] = 0
        self.servoCmds['doorClosed'] = 100
        self.servoCmds['doorOpen'] = 0


    '''
    Table for how servos should be contrlled for desired position


            Servo              Starting Position                  End Position              
    ================================================================================
        gripperRotate   |            0                  |              90
        gripperClamp    |            0                  |              180
            wrist       |           180                 |            variable
            arm         |           180                 |               0
           paddle       |           180                 |               0 
            door        |           100                 |               0


    '''

    #This will receive the corresponding key

    def sendMsg(self, msg, wristAdjust = 180):
        self.servoCmds['wristAdjust'] = wristAdjust
        self.pub.publish(self.servoCmds[msg])

    def callBack(self, msg):
        if msg.data == 0:
            self.taskComplete = True
        else:
            self.taskComplete = False

    
        

        



