#!/usr/bin/env python3
#
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# Modified by Juan Suquilanda
# Modified by: Ronnie Mohapatra
# Modified by: Jhonny Velasquez

onJetson = False

sim = False

import time
import numpy as np
import rospy

from sensor_msgs.msg import Range
from Ranging import Ranging
from RobotCommand import RobotCommand
from Servos import Servos
if onJetson:
    from Cameras import RealSense

import threading


#This will be the main class that inherits everything from every other class
class Robot:
    def __init__(self, robot_name, node_name, command_topic, queue_size, servos = {}):
        
        rospy.init_node("%s_%s" %(robot_name, node_name), anonymous=True)

        if onJetson:
            self.realSense = RealSense()

        self.ctrl = RobotCommand(robot_name, node_name, command_topic, queue_size = 10)
        self.rng = Ranging(robot_name, node_name)
        

        self.gripperRotate = Servos(robot_name, node_name, "gripperRotate", queue_size = 10)
        self.gripperClamp = Servos(robot_name, node_name, "gripperClamp", queue_size = 10)
        self.door = Servos(robot_name, node_name, "door", queue_size = 10)
        self.arm = Servos(robot_name, node_name, "arm", queue_size = 10)
        self.wrist = Servos(robot_name, node_name, "wrist", queue_size = 10)
        self.paddle = Servos(robot_name, node_name, "paddle", queue_size = 10)
        self.lifting = Servos(robot_name, node_name, "lifting", queue_size = 10)
        self.carousel = Servos(robot_name, node_name, "carousel", queue_size = 10)


        self.currYawAngle = 0.0 - 180.0 #This will ensure our starting yaw will be 0 degees easier calculations
        self.initBoardWidth = 233.0
        self.initYaw = 0.0
        self.botWidth = 30.0#!/usr/bin/env python3
#
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# Modified by Juan Suquilanda
# Modified by: Ronnie Mohapatra
# Modified by: Jhonny Velasquez

onJetson = False

sim = True

import time
import numpy as np
import rospy

from sensor_msgs.msg import Range
from Ranging import Ranging
from RobotCommand import RobotCommand
from Servos import Servos
if onJetson:
    from Cameras import RealSense


#This will be the main class that inherits everything from every other class
class Robot:
    def __init__(self, robot_name, node_name, command_topic, queue_size, servos = {}):
        
        rospy.init_node("%s_%s" %(robot_name, node_name), anonymous=True)

        if onJetson:
            self.realSense = RealSense()

        self.ctrl = RobotCommand(robot_name, node_name, command_topic, queue_size = 10)
        self.rng = Ranging(robot_name, node_name)
        

        self.gripperRotate = Servos(robot_name, node_name, "gripperRotate", queue_size = 10)
        self.gripperClamp = Servos(robot_name, node_name, "gripperClamp", queue_size = 10)
        self.door = Servos(robot_name, node_name, "door", queue_size = 10)
        self.arm = Servos(robot_name, node_name, "arm", queue_size = 10)
        self.wrist = Servos(robot_name, node_name, "wrist", queue_size = 10)
        self.paddle = Servos(robot_name, node_name, "paddle", queue_size = 10)
        self.lifting = Servos(robot_name, node_name, "lifting", queue_size = 10)
        self.carousel = Servos(robot_name, node_name, "carousel", queue_size = 10)


        self.currYawAngle = 0.0 - 180.0 #This will ensure our starting yaw will be 0 degees easier calculations
        self.initBoardWidth = 233.0
        self.initYaw = 0.0
        self.botWidth = 30.0

    #Adding Helper functions to make it easy and clearn to align bot on what ever side we want

    def initServos(self):
        self.arm.sendMsg('armDown')

    #Going to write the function that will make the robot go left and grab objects along the way
    def pickupPathLeft(self):

        while not bot.rng.getLeft(0) <= 10 and not bot.rng.getLeft(1) <= 10:
            while not bot.rng.getLeft(0) <= 10:
                if bot.realSense.getObjDetect(0) == 1:
                    break
                bot.ctrl.goLeft(0.5)
            
            bot.ctrl.stopBot()

            if bot.rng.getObjDetect()[0] == 1:
                bot.cameraAlign()
                bot.tofApproach()
                bot.tofAllign()
                while not bot.rng.getBack(0) <= 30 and not bot.rng.getBack(1) <= 30:
                    bot.ctrl.goBackwards(0.5)
                bot.ctrl.stopBot()
    
    def pickupPathRight(self):
        while not bot.rng.getRight(0) <= 10 and not bot.rng.getRight(1) <= 10:
            while not bot.rng.getRight(0) <= 10:
                if bot.realSense.getObjDetect(0) == 1:
                    break
                bot.ctrl.goRight(0.5)
            
            bot.ctrl.stopBot()

            if bot.rng.getObjDetect()[0] == 1:
                bot.cameraAlign()
                bot.tofApproach()
                bot.tofAllign()
                while not bot.rng.getBack(0) <= 30 and not bot.rng.getBack(1) <= 30:
                    bot.ctrl.goBackwards(0.5)
                bot.ctrl.stopBot()
        
    def pickUprightPedestal(self):
        print("Arm Down")
        bot.arm.sendMsg('armDown')
        time.sleep(2)
        print('Rotating Default')
        bot.gripperRotate.sendMsg('gripperRotateDefault')
        time.sleep(1)
        print("Opening")
        bot.gripperClamp.sendMsg('gripperClampOpen')
        time.sleep(1)
        print("Closing")
        bot.gripperClamp.sendMsg('gripperClampClosed')
        time.sleep(1)
        print("Arm Going Up")
        bot.arm.sendMsg('armUp')
        time.sleep(2)
        print('Rotating')
        bot.gripperRotate.sendMsg('gripperRotate90')
        time.sleep(1)
        print('Opening')
        bot.gripperClamp.sendMsg('gripperClampOpen')

    def tofApproach(self):
        self.rng.getTofSensors(0)
        self.rng.getTofSensors(1)


        #We are going to go fowrward until we detect a disturbance for TOF
        print("Sensor Values", self.rng.getTofSensors())
        while(self.rng.getTofSensors(0) > 11.0 or self.rng.getTofSensors(1) > 11.0):
            self.ctrl.goFoward(.1)

        #As soon as we detect a disturbance stop the bot
        self.ctrl.stopBot()

    def cameraAlign(self):
        data = self.realSense.getObjDetect()
        while data[0] == 1 and (data[1] < 200 or data[1] > 400):
            if data[1] < 200:
                self.ctrl.goLeft(0.25)
            else:
                self.ctrl.goRight(0.25)
            data = self.realSense.getObjDetect()
            
        print("stopping")
        self.ctrl.stopBot()


    def tofAllign(self):

        print("Alligning")

        threshhold = 1

        while (abs(self.rng.getTofSensors(0) - self.rng.getTofSensors(1)) > threshhold):
            print("Sensor Readings: " ,self.rng.getTofSensors())
            if self.rng.getTofSensors(0) > self.rng.getTofSensors(1):
                self.ctrl.goRight(0.05)
            else:
                self.ctrl.goLeft(0.05)
        self.ctrl.stopBot()

    def pickup(self):
        pass
    

    def alignFront(self, threshhold = 0.75):
        while abs(self.rng.getFront(0) - self.rng.getFront(1)) > threshhold:
            if self.rng.getFront(0) > self.rng.getFront(1):
                self.ctrl.rotateRight()
            else:
                self.ctrl.rotateLeft()
        self.ctrl.stopBot()

    def alignRight(self, threshhold = 0.75):
        while abs(self.rng.getRight(0) - self.rng.getRight(1)) > threshhold:
            if self.rng.getLeft(0) > self.rng.getRight(1):
                self.ctrl.rotateLeft(0.25)
            else:
                self.ctrl.rotateRight(0.25)
        self.ctrl.stopBot()

    def alignBack(self, threshhold = 0.75):
        while abs(self.rng.getBack(0) - self.rng.getBack(1)) > threshhold:
            if self.rng.getBack(0) > self.rng.getBack(1):
                self.ctrl.rotateLeft(0.25)
            else:
                self.ctrl.rotateRight(0.25)
        self.ctrl.stopBot()

    def alignLeft(self, threshhold = 0.75):
        while abs(self.rng.getLeft(0) - self.rng.getLeft(1)) > threshhold:
            if self.rng.getLeft(0) > self.rng.getLeft(1):
                self.ctrl.rotateLeft(0.25)
            else:
                self.ctrl.rotateRight(0.25)
        self.ctrl.stopBot()


    def initStartingConditions(self):
        time.sleep(2)
        self.initYaw = self.currYawAngle
        self.initBoardWidth = self.rng.getLeft(1) + self.botWidth + self.rng.getRight(0)
        print("Initial Board Width", self.initBoardWidth)

    def goToLocationC(self):
        #It is important that in the begining of the wround the intial yaw value is saved
        #For this to happen out goal state will be dependent on back and right side sensors
        #Therefore we will create variables that we would expect to read with the ultrasonic sensors

        #TODO measure the actual values and plug into here
        #c_x is the value we want to get from the back sensor
        #c_y is the value we want to get from the right sensor
        c_x = 30.0
        c_y = 64.0

        #First we are going to make sure that the robot has the same yaw that it had in the begining
        #This will ensure that the sensors will be parallel to their opossing wall

        currYaw = self.currYawAngle
    
        '''
        Now we know how much we should rotate. Since a clockwise rotation increases yaw, and 
        counter clockwise decreases yaw. If the mod of of the current 
        '''
        #This means we will have to rotate right

        '''
        if currYaw < 0:
            yawOffset = (currYaw % 360) - 360
            while (self.currYawAngle < currYaw + abs(yawOffset)):
                self.rotateRight()

        elif (currYaw > 0):
            while(self.currYaw > currYaw - yawOffset):
                self.rotateLeft()
        '''
        #Now we should (within1inch(self.rng.getRight(1), c_y), 2)be in the same orientation we started in and can start moving in the direction we need to go
        #We will need to build a custom message since our location can be anywhere on the board

        #We'll take a moment and let values comes in
        #Need to test which sensor from back is more reliable
        if self.rng.getBack(0) < c_x :
            msg_x = c_x - self.rng.getBack(0)
        else:
            msg_x = -(self.rng.getBack(0) - c_x)
        msg_y = 0.0

        #This condition checks to see which sensors are closer to wall therefore we can rely on them better
        if self.rng.getLeft(0) > self.rng.getRight(0):
            msg_y = self.rng.getRight(0) - c_y
        else:
            msg_y = self.initBoardWidth - self.rng.getLeft(1) - c_y
        
        #Now we should have the vector we need to travel in for robot to get to location
        msg = self.ctrl.buildMsg(msg_x, msg_y, 0, 0.25)

        while not within1inch(self.rng.getRight(0), c_y, 3)  :
            self.ctrl.sendMsg(msg)
        print("Exit Conditions: Right: ", self.rng.getRight(), " Back: ", self.rng.getBack())

        #Now that we got close we are going to align bot with back
        
        self.ctrl.stopBot()
        #Now we should be at or near location


#Put helper functions here prob will make a util class later

def within1inch(n, target, threshold=1):
    # Convert threshold to centimeters

    if n >= target - threshold and n <= target + threshold:
        return True
    else:
        return False


if __name__ == "__main__":

    

    if sim:
        bot = Robot("sim","talker","cmd_vel", queue_size = 10)
    else:
        bot = Robot("bot","talker","cmd_vel", queue_size = 10)
    '''
    bot.stopBot()
    #delay program for 5 seconds
    #Need to wait for yaw angles to actually come in so will prob have to put in a delay somewhere in here
    bot.initStartingConditions()
    print("Right: ", bot.ultraRight, "Back: ", bot.ultraBack)
    print("turn on motors")
    currTime = time.time()
    currTime = time.time()

    while True:
        print(bot.realSense.getDetectionData())
    
    '''

    bot.pickupPathLeft()
    bot.pickupPathRight()
   
    
    
 
