# !/usr/bin/env python3
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
from std_msgs.msg import Int8

if onJetson:
    from Cameras import RealSense
from pedestal_classification.PedestalTracker import PedestalTracker


# This will be the main class that inherits everything from every other class
class Robot:
    def __init__(self, robot_name, node_name, command_topic, queue_size, servos={}):

        rospy.init_node("%s_%s" % (robot_name, node_name), anonymous=True)

        if onJetson:
            self.realSense = RealSense()

        self.ctrl = RobotCommand(robot_name, node_name, command_topic, queue_size=10)
        self.rng = Ranging(robot_name, node_name)

        self.gripperRotate = Servos(robot_name, node_name, "gripperRotate", queue_size=10)
        self.gripperClamp = Servos(robot_name, node_name, "gripperClamp", queue_size=10)
        self.door = Servos(robot_name, node_name, "door", queue_size=10)
        self.arm = Servos(robot_name, node_name, "arm", queue_size=10)
        self.wrist = Servos(robot_name, node_name, "wrist", queue_size=10)
        self.paddle = Servos(robot_name, node_name, "paddle", queue_size=10)
        self.lifting = Servos(robot_name, node_name, "lifting", queue_size=10)
        self.carousel = Servos(robot_name, node_name, "carousel", queue_size=10)
        rospy.Subscriber('%s/redled' % robot_name, Int8, self.redled)

        self.currYawAngle = 0.0 - 180.0  # This will ensure our starting yaw will be 0 degees easier calculations
        self.initBoardWidth = 233.0
        self.initYaw = 0.0
        self.botWidth = 30.0
        self.led = 0

    # Adding Helper functions to make it easy and clearn to align bot on what ever side we want
    def redled(self, msg):
        self.led = msg.data

    def getRedLed(self):
        # rotate 90 degrees if self.led = 1 otherwise, led is not detected
        return self.led

    def initServos(self):

        self.lifting.sendMsg('liftUp')
        time.sleep(3)
        self.arm.sendMsg('armDown')
        time.sleep(2)
        self.gripperClamp.sendMsg('gripperClampClosed')
        time.sleep(1)
        self.wrist.sendMsg('wristDefault')

    # Going to write the function that will make the robot go left and grab objects along the way
    def pickupPathLeft(self):
        while not self.rng.getLeft(0) <= 15 and not self.rng.getLeft(1) <= 15:
            while not self.rng.getLeft(0) <= 10:
                if self.rng.getObjDetect()[0] == 1:
                    break
                self.ctrl.goLeft(0.5)

            self.ctrl.stopBot()

            if self.rng.getObjDetect()[0] == 1:
                self.cameraAlign()
                self.tofApproach()
                self.tofAllign()
                self.ctrl.stopBot(3)
                self.pickUprightPedestal()

                while not self.rng.getBack(0) <= 30 and not self.rng.getBack(1) <= 30:
                    self.ctrl.goBackwards(0.5)
                self.ctrl.stopBot()
                # self.alignBack()

    def pickupPathRight(self):
        while not self.rng.getRight(0) <= 10 and not self.rng.getRight(1) <= 10:
            while not self.rng.getRight(0) <= 10:
                if self.realSense.getObjDetect(0) == 1:
                    break
                self.ctrl.goRight(0.5)

            self.ctrl.stopBot()

            if self.rng.getObjDetect()[0] == 1:
                self.cameraAlign()
                self.tofApproach()
                self.tofAllign()
                while not self.rng.getBack(0) <= 30 and not self.rng.getBack(1) <= 30:
                    self.ctrl.goBackwards(0.5)
                self.ctrl.stopBot()

    def pickUprightPedestal(self):
        print("Arm Down")
        self.arm.sendMsg('armDown')
        time.sleep(3)
        print('Rotating Default')
        self.gripperRotate.sendMsg('gripperRotateDefault')
        time.sleep(1)
        print("Opening")
        self.gripperClamp.sendMsg('gripperClampOpen')
        time.sleep(2)
        print("Lift Down")
        self.lifting.sendMsg('liftDown')
        time.sleep(2)
        print("Closing")
        self.gripperClamp.sendMsg('gripperClampClosed')
        time.sleep(2)
        print("Lift Up")
        self.lifting.sendMsg('liftUp')
        time.sleep(4)
        print("Arm Going Up")
        self.arm.sendMsg('armUp')
        time.sleep(2)
        print('Rotating')
        self.gripperRotate.sendMsg('gripperRotate90')
        time.sleep(1)
        print('Opening')
        self.gripperClamp.sendMsg('gripperClampOpen')
        time.sleep(1)
        print('Closing')
        self.gripperClamp.sendMsg('gripperClampClosed')

    def tofApproach(self):
        # We are going to go fowrward until we detect a disturbance for TOF
        print("Sensor Values", self.rng.getTofSensors())
        while (self.rng.getTofSensors(1) > 12.5):
            self.ctrl.goFoward(.1)

        # As soon as we detect a disturbance stop the bot
        self.ctrl.stopBot()

    def cameraAlign(self):
        data = self.rng.getObjDetect()
        while data[0] == 1 and (data[1] < 150 or data[1] > 275):
            if data[1] < 150:
                self.ctrl.goLeft(0.25)
            else:
                self.ctrl.goRight(0.25)
            data = self.rng.getObjDetect()

        print("stopping")
        self.ctrl.stopBot()

    def tofAllign(self):

        print("Alligning")

        threshhold = 0.1

        while (self.rng.getTofSensors(1) < 4.5 or self.rng.getTofSensors(1) > 7):
            if self.rng.getTofSensors(1) < 4.5:
                self.ctrl.goLeft(0.1)
            else:
                self.ctrl.goRight(0.1)
        self.ctrl.stopBot()

    def pickup(self):
        pass

    def alignFront(self, threshhold=0.75):
        while abs(self.rng.getFront(0) - self.rng.getFront(1)) > threshhold:
            if self.rng.getFront(0) > self.rng.getFront(1):
                self.ctrl.rotateRight()
            else:
                self.ctrl.rotateLeft()
        self.ctrl.stopBot()

    def alignRight(self, threshhold=0.75):
        while abs(self.rng.getRight(0) - self.rng.getRight(1)) > threshhold:
            if self.rng.getLeft(0) > self.rng.getRight(1):
                self.ctrl.rotateLeft(0.25)
            else:
                self.ctrl.rotateRight(0.25)
        self.ctrl.stopBot()

    def alignBack(self, threshhold=0.75):
        while abs(self.rng.getBack(0) - self.rng.getBack(1)) > threshhold:
            if self.rng.getBack(0) > self.rng.getBack(1):
                self.ctrl.rotateLeft(0.25)
            else:
                self.ctrl.rotateRight(0.25)
        self.ctrl.stopBot()

    def alignLeft(self, threshhold=0.75):
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

        '''
        Now we know how much we should rotate. Since a clockwise rotation increases yaw, and 
        counter clockwise decreases yaw. If the mod of of the current 
        '''
        # This means we will have to rotate right

        '''
        if currYaw < 0:
            yawOffset = (currYaw % 360) - 360
            while (self.currYawAngle < currYaw + abs(yawOffset)):
                self.rotateRight()

        elif (currYaw > 0):
            while(self.currYaw > currYaw - yawOffset):
                self.rotateLeft()
        A - Right statue
        B - Left statue
        C - Duck Pond
        '''

    def goToLocationA(self):
        # It is important that in the begining of the wround the intial yaw value is saved
        # For this to happen out goal state will be dependent on back and right side sensors
        # Therefore we will create variables that we would expect to read with the ultrasonic sensors

        # TODO measure the actual values and plug into here
        # B_x is the value we want to get from the back sensor
        # B_y is the value we want to get from the right sensor
        print("Goint To Location A")
        A_x = 30.0
        A_y = 64.0

        # First we are going to make sure that the robot has the same yaw that it had in the begining
        # This will ensure that the sensors will be parallel to their opossing wall

        currYaw = self.currYawAngle

        # Now we should (within1inch(self.rng.getRight(1), B_y), 2)be in the same orientation we started in and can start moving in the direction we need to go
        # We will need to build a custom message since our location can be anywhere on the board

        # We'll take a moment and let values comes in
        # Need to test which sensor from back is more reliable
        if self.rng.getBack(1) < A_x:
            msg_x = A_x - self.rng.getBack(1)
        else:
            msg_x = -(self.rng.getBack(1) - A_x)
        msg_y = 0.0

        # This condition checks to see which sensors are closer to wall therefore we can rely on them better
        if self.rng.getRight(0) > self.rng.getLeft(0):
            msg_y = A_y - self.rng.getLeft(0)
        else:
            msg_y = -(self.initBoardWidth - self.rng.getRight(1) - A_y)

        # Now we should have the vector we need to travel in for robot to get to location
        msg = self.ctrl.buildMsg(msg_x, -msg_y, 0, 0.25)
        print("MSG X-Y Components: ", msg_x, msg_y)

        while not within1inch(self.rng.getLeft(0), A_y, 3):
            self.ctrl.sendMsg(msg)
        print("Exit Conditions: Right: ", self.rng.getLeft(), " Back: ", self.rng.getBack())

        # Now that we got close we are going to align bot with back

        self.ctrl.stopBot()
        # Now we should be at or near location

    def goToLocationB(self):
        # It is important that in the begining of the wround the intial yaw value is saved
        # For this to happen out goal state will be dependent on back and right side sensors
        # Therefore we will create variables that we would expect to read with the ultrasonic sensors

        # TODO measure the actual values and plug into here
        # B_x is the value we want to get from the back sensor
        # B_y is the value we want to get from the right sensor
        print("Goint To Location B")
        B_x = 30.0
        B_y = 64.0

        # First we are going to make sure that the robot has the same yaw that it had in the begining
        # This will ensure that the sensors will be parallel to their opossing wall

        currYaw = self.currYawAngle

        '''
        Now we know how much we should rotate. Since a clockwise rotation increases yaw, and 
        counter clockwise decreases yaw. If the mod of of the current 
        '''
        # This means we will have to rotate right

        '''
        if currYaw < 0:
            yawOffset = (currYaw % 360) - 360
            while (self.currYawAngle < currYaw + abs(yawOffset)):
                self.rotateRight()

        elif (currYaw > 0):
            while(self.currYaw > currYaw - yawOffset):
                self.rotateLeft()
        '''
        # Now we should (within1inch(self.rng.getRight(1), B_y), 2)be in the same orientation we started in and can start moving in the direction we need to go
        # We will need to build a custom message since our location can be anywhere on the board

        # We'll take a moment and let values comes in
        # Need to test which sensor from back is more reliable
        if self.rng.getBack(1) < B_x:
            msg_x = B_x - self.rng.getBack(1)
        else:
            msg_x = -(self.rng.getBack(1) - B_x)
        msg_y = 0.0

        # This condition checks to see which sensors are closer to wall therefore we can rely on them better
        if self.rng.getLeft(0) > self.rng.getRight(0):
            msg_y = self.rng.getRight(0) - B_y
        else:
            msg_y = (self.initBoardWidth - self.rng.getLeft(1) - B_y)

        # Now we should have the vector we need to travel in for robot to get to location
        msg = self.ctrl.buildMsg(msg_x, -msg_y, 0, 0.25)
        print("MSG X-Y Components: ", msg_x, msg_y)

        while not within1inch(self.rng.getRight(0), B_y, 3):
            self.ctrl.sendMsg(msg)
        print("Exit Conditions: Right: ", self.rng.getRight(), " Back: ", self.rng.getBack())

        # Now that we got close we are going to align bot with back

        self.ctrl.stopBot()
        # Now we should be at or near location

    def goToLocationC(self):
        # It is important that in the begining of the wround the intial yaw value is saved
        # For this to happen out goal state will be dependent on back and right side sensors
        # Therefore we will create variables that we would expect to read with the ultrasonic sensors

        # TODO measure the actual values and plug into here
        # B_x is the value we want to get from the back sensor
        # B_y is the value we want to get from the right sensor
        print("Goint To Location C")
        C_x = 50.0
        C_y = 100.0

        # First we are going to make sure that the robot has the same yaw that it had in the begining
        # This will ensure that the sensors will be parallel to their opossing wall

        currYaw = self.currYawAngle

        '''
        Now we know how much we should rotate. Since a clockwise rotation increases yaw, and 
        counter clockwise decreases yaw. If the mod of of the current 
        '''
        # This means we will have to rotate right

        '''
        if currYaw < 0:
            yawOffset = (currYaw % 360) - 360
            while (self.currYawAngle < currYaw + abs(yawOffset)):
                self.rotateRight()

        elif (currYaw > 0):
            while(self.currYaw > currYaw - yawOffset):
                self.rotateLeft()
        '''
        # Now we should (within1inch(self.rng.getRight(1), B_y), 2)be in the same orientation we started in and can start moving in the direction we need to go
        # We will need to build a custom message since our location can be anywhere on the board

        # We'll take a moment and let values comes in
        # Need to test which sensor from back is more reliable
        if self.rng.getBack(1) < C_x:
            msg_x = C_x - self.rng.getBack(1)
        else:
            msg_x = -(self.rng.getBack(1) - C_x)
        msg_y = 0.0

        # This condition checks to see which sensors are closer to wall therefore we can rely on them better
        if self.rng.getLeft(0) > self.rng.getRight(0):
            msg_y = self.rng.getRight(0) - C_y
        else:
            msg_y = C_y - self.rng.getLeft(0)

        # Now we should have the vector we need to travel in for robot to get to location
        msg = self.ctrl.buildMsg(msg_x, -msg_y, 0, 0.25)
        print("MSG X-Y Components: ", msg_x, msg_y)

        while not within1inch(self.rng.getRight(0), C_y, 2) and not within1inch(self.rng.getLeft(0), C_y, 2):
            self.ctrl.sendMsg(msg)
        print("Exit Conditions: Right/Left: ", self.rng.getRight(), "/", self.rng.getRight(), " Back: ",
              self.rng.getBack())

        # Now that we got close we are going to align bot with back

        self.ctrl.stopBot()
        # Now we should be at or near location

    def alignDispensing(self):
        dropofflocation = False
        start = time.time()
        #        while(!self.color.isWhite()):
        while dropofflocation is False and (time.time() - start < 2000):
            if self.color.isWhite():
                # stop robot
                self.ctrl.stopBot()
                dropofflocation = True
            # else go right
            else:
                RobotCommand.goRight()
        if not dropofflocation:
            if self.color.isWhite():
                # stop robot
                self.ctrl.stopBot()
                dropofflocation = True
            # else go forward
            else:
                self.ctrl.goFoward()
        start = time.time()
        while dropofflocation is False and (time.time() - start < 2000):
            if self.color.isWhite():
                # stop robot
                self.ctrl.stopBot()
                dropofflocation = True
            # else go left
            else:
                self.ctrl.goLeft()
        if not dropofflocation:
            if self.color.isWhite():
                # stop robot
                self.ctrl.stopBot()
                dropofflocation = True
            # else go forward
            else:
                self.ctrl.goFoward()


# Put helper functions here prob will make a util class later

def within1inch(n, target, threshold=1):
    # Convert threshold to centimeters

    if n >= target - threshold and n <= target + threshold:
        return True
    else:
        return False


if __name__ == "__main__":

    if sim:
        bot = Robot("sim", "talker", "cmd_vel", queue_size=10)
    else:
        bot = Robot("bot", "talker", "cmd_vel", queue_size=10)
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

    # bot.pickupPathLeft()
    # bot.pickupPathRight()
    time.sleep(2)
    bot.goToLocationA()
    # bot.goToLocationB()
    # bot.goToLocationC()
    # bot.goToLocationC()


