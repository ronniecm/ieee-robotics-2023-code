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


class RobotCommand:
    def __init__(self, robot_name, node_name, command_topic, queue_size):
        

        #Using .pub() to publish messages to robot
        self.robot_name = robot_name
        
        self.command_topic = command_topic
        self.pub = rospy.Publisher("%s/%s" %(robot_name,command_topic), Twist, queue_size =10)
           
        
    def yaw(self, msg):
        self.currYawAngle = msg.data
    
    def obj_detect(self, msg):
        self.objDetect = msg.range * 100.0
        
    
    def buildMsg(self, x, y, rot, speed = 1):
        msg = Twist()
        
        print("Building msg with following params: %s,%s,%s" %(x,y,rot))

        #We will need toprint("Before params: ", msg) make a unit vector for this this will normalize to 1
        mag = np.sqrt(pow(x,2) + pow(y,2))

        if mag == 0.0:
            norm = 1
        else:
            norm = 1/mag
        
        msg.linear.x = x*norm * speed
        msg.linear.y = -1*y*norm * speed
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

    def goFoward(self):
        # Move the robot forward
        msg = self.buildMsg(1.0, 0.0, 0.0)
        self.pub.publish(msg)
        
    def goBackwards(self, speed = 1):
        # Move the robot backwards
        msg = self.buildMsg(-1.0*speed, 0.0, 0.0)
        self.pub.publish(msg)
        
    def goRight(self, speed = 1):
        # Move the robot right
        msg = self.buildMsg(0, 1.0*speed, 0.0)
        self.pub.publish(msg)
        
    def goLeft(self, speed = 1):
        # Move the robot left
        msg = self.buildMsg(0.0, -1.0*speed, 0.0)
        self.pub.publish(msg)

    # Rotate the robot left
    def rotateLeft(self, speed = 1):
        msg = self.buildMsg(0.0, 0.0, -1.0*speed)
        self.pub.publish(msg)

    def rotateRight(self, speed = 1):
        # Rotate the robot right
        msg = self.buildMsg(0.0, 0.0, 1*speed)
        self.pub.publish(msg)
        
    def stopBot(self, time = 0):
        # Stop the robot
        msg = self.buildMsg(0.0, 0.0, 0.0)
        self.pub.publish(msg)
        if time == 0:
            pass
        else:
            time.sleep(time)

    #Adding Helper functions to make it easy and clearn to align bot on what ever side we want

    def alignFront(self, threshhold = 0.75):
        while abs(self.rng.getFront(0) - self.rng.getFront(1)) > threshhold:
            if self.rng.getFront(0) > self.rng.getFront(1):
                self.rotateRight()
            else:
                self.rotateLeft()

    def alignRight(self, threshhold = 0.75):
        while abs(self.rng.getRight(0) - self.rng.getRight(1)) > threshhold:
            if self.rng.getRight(0) > self.rng.getRight(1):
                self.rotateRight(0.25)
            else:
                self.rotateLeft(0.25)

    def alignBack(self, threshhold = 0.75):
        while abs(self.rng.getBack(0) - self.rng.getBack(1)) > threshhold:
            if self.rng.getBack(0) > self.rng.getBack(1):
                self.rotateRight(0.25)
            else:
                self.rotateLeft(0.25)

    def alignLeft(self, threshhold = 0.75):
        while abs(self.rng.getLeft(0) - self.rng.getLeft(1)) > threshhold:
            if self.rng.getLeft(0) > self.rng.getLeft(1):
                self.rotateRight(0.25)
            else:
                self.rotateLeft(0.25)

    '''
    Each face of the robot has a list assigned to it. You can access the sensors on that face through a list
    That list is will be self.rng.get<FACE>
    So if I wanted to get sensor readings from the sensor on the front left I would call self.rng.getFront(0)
    
    0 index corresponds to a left sensor on a face and 1 corresponds to the right sensor on the face

    Make sure this indexing is consistant througout ALL error correction calculations otherwise there were will unintentional erros
    This is how indexing should work such that function work the same for all sides and no need to repeat any calculations
    If this not working make sure to check the call ultrasonic callback functions to make sure the distances are being indexed correcly I 
    may have messed up !!
                                            -Front-
                                         _0_______1_
                                      1 |           | 0
                                        |           |
                                        |           |
                                      0 |           | 1
                                        |___________|
                                            1       0   
    '''

    
        
    #Creating a function that will get robot to known statue locations labled A, B, C

    
        

