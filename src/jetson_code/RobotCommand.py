#!/usr/bin/env python3

#==========================================
# Title:  Robot Ranging Class
# Author: Juan Suquilanda
# Date:   26 February 2023
#==========================================

'''
Classed used as a modular way of controlling robot
'''

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
import time


class RobotCommand:
    def __init__(self, robot_name, node_name, command_topic, queue_size):
        

        #Using .pub() to publish messages to robot
        self.robot_name = robot_name
        
        self.command_topic = command_topic
        self.pub = rospy.Publisher("%s/%s" %(robot_name,command_topic), Twist, queue_size =10)
           
        
    def yaw(self, msg):
        self.currYawAngle = msg.data
        
    
    def buildMsg(self, x, y, rot, speed = 1):
        msg = Twist()
        
        #print("Building msg with following params: %s,%s,%s" %(x,y,rot))

        #We will need toprint("Before params: ", msg) make a unit vector for this this will normalize to 1
        mag = np.sqrt(pow(x,2) + pow(y,2))

        if mag == 0.0:
            norm = 1
        else:
            norm = 1/mag
        
        msg.linear.x = x*norm * speed
        msg.linear.y = y*norm * speed
        msg.linear.z = 0.0
        
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = rot
        #print("Message Built", msg)
        #print(msg)
        return msg

    def sendMsg(self, msg):
        self.pub.publish(msg)
    
    #List of basic commands that can be used as geometry twist messages

    def goFoward(self,speed = 1):
        # Move the robot forward
        msg = self.buildMsg(1.0*speed, 0.0, 0.0, speed)
        self.pub.publish(msg)
        
    def goBackwards(self, speed = 1):
        # Move the robot backwards
        msg = self.buildMsg(-1.0*speed, 0.0, 0.0, speed)
        self.pub.publish(msg)
        
    def goRight(self, speed = 1):
        # Move the robot right
        msg = self.buildMsg(0, 1.0*speed, 0.0, speed)
        self.pub.publish(msg)
        
    def goLeft(self, speed = 1):
        # Move the robot left
        msg = self.buildMsg(0.0, -1.0*speed, 0.0, speed)
        self.pub.publish(msg)

    # Rotate the robot left
    def rotateLeft(self, speed = 1):
        msg = self.buildMsg(0.0, 0.0, -1.0*speed, speed)
        self.pub.publish(msg)

    def rotateRight(self, speed = 1):
        # Rotate the robot right
        msg = self.buildMsg(0.0, 0.0, 1*speed, speed)
        self.pub.publish(msg)
        
    def stopBot(self, delay = 1):
        # Stop the robot
        msg = self.buildMsg(0.0, 0.0, 0.0)

        currTime = time.time()
        while(time.time() <= currTime + delay):
            self.pub.publish(msg)
            
    def testCommands(self):

        currTime = time.time()
        while(time.time() < currTime + 1):
            self.goFoward()

        currTime = time.time()
        while(time.time() < currTime + 1):
            self.goBackwards()
        
        currTime = time.time()
        while(time.time() < currTime + 1):
            self.goRight()

        currTime = time.time()
        while(time.time() < currTime + 1):
            self.goLeft()

        currTime = time.time()
        while(time.time() < currTime + 1):
            self.rotateRight()

        currTime = time.time()

        while(time.time() < currTime + 2):
            self.rotateLeft()

        self.stopBot(1.0)



    
        

