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

class Servos:
    def __init__(self, robot_name, node_name, command_topic, queue_size):
        self.robot_name = robot_name
        
        self.pub = rospy.Publisher("%s/%s_cmd" %(robot_name,command_topic), Int8, queue_size =10)
        # This is for the subscriber
        
        rospy.Subscriber('%s/%s_%s' %(robot_name, command_topic,"callback"), Int8, self.callBack)

        self.actionComplete = False

        

    def sendMsg(self, msg):
        self.pub.publish(msg)

    def callBack(self, msg):
        if msg.data == 0:
            self.taskComplete = True
        else:
            self.taskComplete = False
        

        



