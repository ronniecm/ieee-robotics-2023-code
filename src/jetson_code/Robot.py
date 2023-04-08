
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

onJetson = True

sim = False
import matplotlib.pyplot as plt
import time
import numpy as np
import rospy

from sensor_msgs.msg import Range

from RobotCommand import RobotCommand
from Servos import Servos
from pedestal_classification.PedestalTracker import PedestalTracker


if onJetson:
    from Cameras import RealSense

'''
from Color import Color
from Color import LED
from Ranging import Ranging
'''
from Sensors import Color, LED, Ranging


#This will be the main class that inherits everything from every other class
class Robot:
    def __init__(self, robot_name, node_name, command_topic, queue_size, servos = {}):
        
        rospy.init_node("%s_%s" %(robot_name, node_name), anonymous=True)
        
        if onJetson:
            self.realSense = RealSense()

        self.ctrl = RobotCommand(robot_name, node_name, command_topic, queue_size = 10)
        self.rng = Ranging(robot_name, node_name)
        self.color = Color(robot_name)
        self.led = LED(robot_name)

        # Initialize pedestal tracker object
        #path = "/home/mdelab/ieee-robotics-2023-code/src/jetson_code/pedestal_classification/lightweight_net_color_orientation_v4.pth"
        #path = "/home/mdelab/ieee-robotics-2023-code/src/jetson_code/pedestal_classification/lightweight_net_color_orientation.pth"

        #path = "/home/mdelab/ieee-robotics-2023-code/src/jetson_code/pedestal_classification/lw_net_color_orientation_infer.ipynb"
        #path = '/home/mdelab/ieee-robotics-2023-code/src/jetson_code/pedestal_classification/mediumweight_net_color_orientation_2023-04-07_03-38-59.pth'
        path = '/home/mdelab/ieee-robotics-2023-code/src/jetson_code/pedestal_classification/mediumweight_net_color_orientation_2023-04-07_22-00-45.pth'
        self.pedestal_tracker = PedestalTracker(path, "cpu")
        
        self.gripperRotate = Servos(robot_name, node_name, "gripperRotate", queue_size = 10)
        self.gripperClamp = Servos(robot_name, node_name, "gripperClamp", queue_size = 10)
        self.door = Servos(robot_name, node_name, "door", queue_size = 10)
        self.arm = Servos(robot_name, node_name, "arm", queue_size = 10)
        self.wrist = Servos(robot_name, node_name, "wrist", queue_size = 10)
        self.paddle = Servos(robot_name, node_name, "paddle", queue_size = 10)
        self.lifting = Servos(robot_name, node_name, "lifting", queue_size = 10)
        self.carousel = Servos(robot_name, node_name, "carousel", queue_size = 10)
        

        #need a node here to tell us what that current YAW is
        self.currYawAngle = 0.0  #This will ensure our starting yaw will be 0 degees easier calculations
        self.initBoardWidth = 233.0
        self.botWidth = 30.0
        self.initYaw = 0.0

    #Adding Helper functions to make it easy and clearn to align bot on what ever side we want

    def initServos(self):
        self.lifting.sendMsg('liftUp')
        time.sleep(3)
        self.arm.sendMsg('armDown')
        #time.sleep(2)
        self.gripperClamp.sendMsg('gripperClampClosed')
        time.sleep(1)
        self.wrist.sendMsg('wristDefault')

    def handleWrist(self):
        angleOffset = self.pedestal_tracker.make_prediction()
        #print("angle in degrees: ", angleOffset)

        # Check if upright or on side
        if angleOffset == None:
            print("No pedestal detected or upright pedestal detected")
        else:
            print("Angle in degrees", angleOffset)


        '''
        if angleOffset == -1:
            self.pickUprightPedestal()
        else:
            self.pickUpDownPedestal(angleOffset)
        '''
        
    def pickupComp(self):
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
                bot.pickUprightPedestal() # picks up the pedestal
                while not bot.rng.getBack(0) <= 30 and not bot.rng.getBack(1) <= 30:
                    bot.ctrl.goBackwards(0.5)
                bot.ctrl.stopBot()
            bot.ctrl.goRight(1)

    #Going to write the function that will make the robot go left and grab objects along the way
    def pickupPathLeft(self):
        print("Starting Path")
        while not self.rng.getLeft(0) <= 15 and not self.rng.getLeft(1) <= 15:
            while not self.rng.getLeft(0) <= 15:
                if self.rng.getObjDetect()[0]== 1:
                    print("detected obj")
                    break
        
                self.ctrl.goLeft(0.5)
            
            self.ctrl.stopBot()

            if self.rng.getObjDetect()[0] == 1:
                self.cameraAlign()
                self.tofApproach()
                self.tofAllign()
                self.ctrl.stopBot(3)
                #Going to check if the object is upright or not

                '''
                angleOffset = self.pedestal_tracker.make_prediction()
                print("angle", angleOffset)

                # Check if upright or on side
                if angleOffset == -1:
                    self.pickUprightPedestal()
                else:
                    self.pickUpDownPedestal(angleOffset)
                '''
                self.pickUprightPedestal()
        
                while not self.rng.getBack(0) <= 30:
                    self.ctrl.goBackwards(0.3)

                self.ctrl.stopBot()
                print("correcting to ", self.initYaw)
                self.alignBack(2)

    def pickupPathRight(self):
        while not self.rng.getRight(0) <= 10 and not self.rng.getRight(1) <= 15:
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
        time.sleep(1)
        print('Rotating Default')
        self.gripperRotate.sendMsg('gripperRotateDefault')
        time.sleep(.5)
        print("Opening")
        self.gripperClamp.sendMsg('gripperClampOpen')
        time.sleep(.25)
        print("Lift Down")
        self.lifting.sendMsg('liftDown')
        time.sleep(4.5)
        print("Closing")
        self.gripperClamp.sendMsg('gripperClampClosed')
        time.sleep(1)
        print("Lift Up")
        self.lifting.sendMsg('liftUp')
        time.sleep(4.5)
        print("Arm Going Up")
        self.arm.sendMsg('armUp')
        time.sleep(1.5)
        print('Rotating')
        self.gripperRotate.sendMsg('gripperRotate90')
        time.sleep(1)
        print('Opening')
        self.gripperClamp.sendMsg('gripperClampOpen')
        time.sleep(1)
        self.arm.sendMsg('armDown')
        print('Bringing Arm Back down')
        time.sleep(1)
        self.carousel.sendMsg('carouselAddPedestal')

        
        
    def pickUpDownPedestal(self, angleOffset):
        print("Arm Down")
        self.arm.sendMsg('armDown')
        time.sleep(1)

        print('Rotating Default')
        self.gripperRotate.sendMsg('gripperRotate90')
        time.sleep(1)

        print("Opening")
        self.gripperClamp.sendMsg('gripperClampOpen')

        print("Making Angle Offset Prediciton for 2 seconds")
        currTime = time.time()
        angleOffset = 0.0
        while(time.time() < currTime + 2):
            angleOffset = self.pedestal_tracker.make_prediction()
        print("Wrist Offset: ", angleOffset)
            
        print("Adjusting Wrist")
        self.wrist.sendMsg('wristAdjust', wristAdjust=(angleOffset))
        time.sleep(1)

        print("Lift Down")
        self.lifting.sendMsg('liftDown')
        time.sleep(4.5)

        print("Closing")
        self.gripperClamp.sendMsg('gripperClampClosed')
        time.sleep(1)

        print("Lift Up")
        self.lifting.sendMsg('liftUp')
        time.sleep(4.5)

        print("Arm Going Up")
        self.arm.sendMsg('armUp')
        time.sleep(1.5)

        print('Opening')
        self.gripperClamp.sendMsg('gripperClampOpen')
        time.sleep(1)

        print("Arm Going Back Down")
        self.arm.sendMsg('armDown')
        time.sleep(1)
        self.carousel.sendMsg('carouselAddPedestal')
        

    def tofApproach(self):
        #We are going to go fowrward until we detect a disturbance for TOF
        print("In  TOF Approach")
        while(self.rng.getTOF() > 12.5):
            self.ctrl.goFoward(0.1)
            ##print("Sensor Values", self.rng.getTOF())
        #As soon as we detect a disturbance stop the bot
        self.ctrl.stopBot()


    def cameraAlign(self):
        data = self.rng.getObjDetect()
        while data[0] == 1 and data[1] < 215:
            self.ctrl.goLeft(0.35)
            data = self.rng.getObjDetect()
        #print("stopping")
        self.ctrl.stopBot()


    def tofAllign(self):

        print("Alligning")

        threshhold = 0.1

        while (self.rng.getTOF() < 4.5 or self.rng.getTOF() > 5.5):
            if self.rng.getTOF() < 4.5:
                self.ctrl.goLeft(0.1)
            else:
                self.ctrl.goRight(0.1)
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
            if self.rng.getRight(0) > self.rng.getRight(1):
                self.ctrl.rotateRight(0.25)
            else:
                self.ctrl.rotateLeft(0.25)
        self.ctrl.stopBot()

    def alignBack(self, threshhold = 1):
        while abs(self.rng.getBack(0) - self.rng.getBack(1)) > threshhold:
            if self.rng.getBack(0) > self.rng.getBack(1):
                self.ctrl.rotateRight(0.2)
            else:
                self.ctrl.rotateLeft(0.2)
        self.ctrl.stopBot()

    def alignLeft(self, threshhold = 0.75):
        while abs(self.rng.getLeft(0) - self.rng.getLeft(1)) > threshhold:
            if self.rng.getLeft(0) > self.rng.getLeft(1):
                self.ctrl.rotateRight(0.25)
            else:
                self.ctrl.rotateLeft(0.25)
        self.ctrl.stopBot()

    def goToLocationA(self):
        #It is important that in the begining of the wround the intial yaw value is saved
        #For this to happen out goal state will be dependent on back and right side sensors
        #Therefore we will create variables that we would expect to read with the ultrasonic sensors

        #TODO measure the actual values and plug into here
        #B_x is the value we want to get from the back sensor
        #B_y is the value we want to get from the right sensor
        print("Goint To Location A")
        A_x = 30.0
        A_y = 60.0

        #First we are going to make sure that the robot has the same yaw that it had in the begining
        #This will ensure that the sensors will be parallel to their opossing wall

        currYaw = self.currYawAngle
    

        #Now we should (within1inch(self.rng.getRight(1), B_y), 2)be in the same orientation we started in and can start moving in the direction we need to go
        #We will need to build a custom message since our location can be anywhere on the board

        #We'll take a moment and let values comes in
        #Need to test which sensor from back is more reliable
        if self.rng.getBack(0) < A_x :
            msg_x = A_x - self.rng.getBack(0)
        else:
            msg_x = -(self.rng.getBack(0) - A_x)
        msg_y = 0.0

        #This condition checks to see which sensors are closer to wall therefore we can rely on them better
        if self.rng.getRight(0) > self.rng.getLeft(0):
            msg_y = A_y - self.rng.getLeft(0)
        else:
            msg_y = -(self.initBoardWidth - self.rng.getRight(1) - A_y)
           
        #Now we should have the vector we need to travel in for robot to get to location
        msg = self.ctrl.buildMsg(msg_x, msg_y, 0, 0.5)
        print("MSG X-Y Components: ", msg_x, msg_y)

        while not abs(A_y - self.rng.getLeft(0)) < 1:
            self.ctrl.sendMsg(msg)
        print("Exit Conditions: Right: ", self.rng.getLeft(), " Back: ", self.rng.getBack())

        #Now that we got close we are going to align bot with back
        
        self.ctrl.stopBot()
        #Now we should be at or near location

    def goToLocationB(self):
        #It is important that in the begining of the wround the intial yaw value is saved
        #For this to happen out goal state will be dependent on back and right side sensors
        #Therefore we will create variables that we would expect to read with the ultrasonic sensors

        #TODO measure the actual values and plug into here
        #B_x is the value we want to get from the back sensor
        #B_y is the value we want to get from the right sensor
        print("Goint To Location B")
        B_x = 30.0
        B_y = 64.0

        #First we are going to make sure that the robot has the same yaw that it had in the begining
        #This will ensure that the sensors will be parallel to their opossing wall

    
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
        #Now we should (within1inch(self.rng.getRight(1), B_y), 2)be in the same orientation we started in and can start moving in the direction we need to go
        #We will need to build a custom message since our location can be anywhere on the board

        #We'll take a moment and let values comes in
        #Need to test which sensor from back is more reliable
        if self.rng.getBack(0) < B_x :
            msg_x = B_x - self.rng.getBack(0)
        else:
            msg_x = -(self.rng.getBack(0) - B_x)
        msg_y = 0.0

        #This condition checks to see which sensors are closer to wall therefore we can rely on them better
        if self.rng.getLeft(0) > self.rng.getRight(1):
            msg_y = self.rng.getRight(0) - B_y
        else:
            msg_y = (self.initBoardWidth - self.rng.getLeft(1) - B_y)
        
        #Now we should have the vector we need to travel in for robot to get to location
        msg = self.ctrl.buildMsg(msg_x, msg_y, 0, 0.5)
        print("MSG X-Y Components: ", msg_x, msg_y)

        while not within1inch(self.rng.getRight(1), B_y, 3)  :
            self.ctrl.sendMsg(msg)
        print("Exit Conditions: Right: ", self.rng.getRight(), " Back: ", self.rng.getBack())

        #Now that we got close we are going to align bot with back
        
        self.ctrl.stopBot()
        #Now we should be at or near location



    def goToLocationC(self):
        #It is important that in the begining of the wround the intial yaw value is saved
        #For this to happen out goal state will be dependent on back and right side sensors
        #Therefore we will create variables that we would expect to read with the ultrasonic sensors

        #TODO measure the actual values and plug into here
        #B_x is the value we want to get from the back sensor
        #B_y is the value we want to get from the right sensor
        print("Goint To Location C")
        C_x = 52.0
        C_y = 90.0

        #First we are going to make sure that the robot has the same yaw that it had in the begining
        #This will ensure that the sensors will be parallel to their opossing wall

    
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
        #Now we should (within1inch(self.rng.getRight(1), B_y), 2)be in the same orientation we started in and can start moving in the direction we need to go
        #We will need to build a custom message since our location can be anywhere on the board

        #We'll take a moment and let values comes in
        #Need to test which sensor from back is more reliable
        if self.rng.getBack(0) < C_x :
            msg_x = C_x - self.rng.getBack(0)
        else:
            msg_x = C_x - self.rng.getBack(0)
        msg_y = 0.0

        #This condition checks to see which sensors are closer to wall therefore we can rely on them better
        msg_y = -1 * (C_y - self.rng.getRight(0))
        
        #Now we should have the vector we need to travel in for robot to get to location
        msg = self.ctrl.buildMsg(msg_x, msg_y, 0, 0.5)
        print("MSG X-Y Components: ", msg_x, msg_y)
        

        while bot.rng.getRight(0) < 100:
            self.ctrl.sendMsg(msg)
        print("Exit Conditions: Right/Left: ", self.rng.getRight(),"/",self.rng.getLeft(), " Back: ", self.rng.getBack())

        #Now that we got close we are going to align bot with back

        self.ctrl.stopBot(2)
    
        self.rotateDegrees(170)

        #self.alignBack()

        
        
        #Now we should be at or near location

    def startRound(self):
        currYaw = self.realSense.getCurrYaw()
        #print("Starting Round")
        #print("current YAW: ", self.realSense.getCurrYaw())
        while (self.realSense.getCurrYaw() > currYaw - 90):
            #print("In loop YAW: ", self.realSense.getCurrYaw())
            self.ctrl.rotateLeft()
        #print("done Rotating")
            
        self.alignBack()

    def run(self):

        while(self.led.getRedLed == 0):
            self.ctrl.stopBot()
        #print("LED DETECTED")
        self.startRound()

    #Put helper functions here prob will make a util class later

    def milestone1(self):
        self.initYaw = self.realSense.getCurrYaw()

        while not bot.rng.getRight(1) < 12.0:
            bot.ctrl.goRight(0.60)
        
        bot.ctrl.stopBot()

        while not bot.rng.getBack(1) > 60.0:
            bot.ctrl.goFoward(0.5) 

        bot.ctrl.stopBot()

        while not bot.rng.getBack(1) < 10.0:
            bot.ctrl.goBackwards(0.5)
        
        bot.ctrl.stopBot()
        #bot.alignBack()
        bot.correctYaw()

        bot.ctrl.stopBot()

        while not bot.rng.getLeft(1) < 10.0:
            pos = bot.rng.getRight(1)
            while bot.rng.getRight(1) < pos + 30.0:
                bot.ctrl.goLeft(0.75)
            bot.ctrl.stopBot()
            while not bot.rng.getBack(1) > 60.0:
                bot.ctrl.goFoward(0.5)
            bot.ctrl.stopBot()
            while not bot.rng.getBack(1) < 10.0:
                bot.ctrl.goBackwards(0.5)
            bot.ctrl.stopBot()

            #bot.alignBack()
            bot.correctYaw()
            bot.ctrl.stopBot()

    def correctToAngle(self, angle):
        print(abs(angle - self.realSense.getCurrYaw()))
        while abs(angle - self.realSense.getCurrYaw()) > 5:
            print(abs(angle - self.realSense.getCurrYaw()))
            if angle > self.realSense.getCurrYaw():
                self.ctrl.rotateRight(0.2)
            else:
                self.ctrl.rotateLeft(0.2)
        self.ctrl.stopBot()
        print("End Yaw", self.realSense.getCurrYaw())

    def rotateDegrees(self, angle):
        currYaw = bot.realSense.getCurrYaw()
        print(currYaw)
        if angle > 0:
            while (bot.realSense.getCurrYaw() < currYaw + angle):
                #print("In loop YAW: ", bot.realSense.getCurrYaw())
                bot.ctrl.rotateRight(0.35)

        if angle < 0:
            while (bot.realSense.getCurrYaw() < currYaw + angle):
                #print("In loop YAW: ", bot.realSense.getCurrYaw())
                bot.ctrl.rotateLeft(0.35)
        bot.ctrl.stopBot()
        print(bot.realSense.getCurrYaw())

    

    def alignDropOff(self, backDistance):
        bot.ctrl.stopBot()
        bot.color.requestColorValues()
        time.sleep(200/1000)
        while not self.color.rgb1OnWhite() and not self.color.rgb2OnWhite():
            currTime = time.time()
            while time.time() < currTime + 0.45:
                self.ctrl.goBackwards(.15)
            self.ctrl.stopBot()
            self.color.requestColorValues()
            time.sleep(200 / 1000)

        self.door.sendMsg("doorOpen")
        while not self.rng.getBack(0) == backDistance:
            if self.rng.getBack(0) > backDistance:
                self.ctrl.goBackwards(0.15)
            else:
                self.ctrl.goFoward(0.15)
        self.ctrl.stopBot()
        print("Made it to location")

    def alignDropOff2(self, ringCenter_y):
        print("Looking for ring")
        isWhite1 = False
        isWhite2 = False

        while self.rng.ultraBack[0] > 13:
            self.ctrl.goBackwards(0.1)

        self.ctrl.stopBot()
        bot.color.requestColorValues()

        if self.color.rgb1OnWhite() and self.color.rgb2OnWhite():
           return
        #If we have made it here then neither are white, or one is white
        isWhite1 = self.color.rgb1OnWhite()
        isWhite2 = self.color.rgb2OnWhite()
        print("isWhite1: ", isWhite1, " isWhite2: ", isWhite2)
        
        while isWhite1 == False or isWhite2 == False:
            '''
            if not isWhite1 and not isWhite2:
                #Check the ultrasonic Readings and move accordingly
                if self.rng.getLeft(0) < ringCenter_y:
                    while self.rng.getLeft(0) < ringCenter_y:
                        self.ctrl.goRight(.1)
                else:
                    while self.rng.getRight(0) < ringCenter_y:
                        self.ctrl.goLeft(.1)
            '''
            #Now we should be closer to the ring and start using the color sensors
            if isWhite1:
                print("We saw White1")
                while not isWhite2:
                    self.ctrl.goRight(.1)
                    bot.crtl.stopBot(.1)
                    self.color.requestColorValues()
                    bot.crtl.stopBot(.1)
                    isWhite2 = self.color.rgb2OnWhite()
                self.ctrl.stopBot()
                return

            if isWhite2:
                print("We saw White2")
                while not isWhite1:
                    self.ctrl.goRight(.1)
                    bot.crtl.stopBot(.1)
                    self.color.requestColorValues()
                    bot.crtl.stopBot(.1)
                    isWhite1 = self.color.rgb1OnWhite()
                self.ctrl.stopBot()
                return
                    
        while self.rng.ultraBack[0] > 11:
            self.ctrl.goBackwards(0.1)
        self.ctrl.stopBot()


    def milestone3(self):
        self.pickupPathLeft()
        time.sleep(2)
        print("Going to Location A")
        self.goToLocationA()
        self.alignDropOff(13)
        currTime = time.time()
        while time.time() < currTime + 1.0:
            self.ctrl.goFoward(0.4)
        self.ctrl.stopself()
        self.door.sendMsg("doorClosed")

        self.carousel.sendMsg("twoStack")
        while not self.rng.getRight(0) <= 66:
            self.ctrl.goRight(0.5)
        self.ctrl.stopself()
        self.alignDropOff(13)
        currTime = time.time()
        while time.time() < currTime + 1.0:
            self.ctrl.goFoward(0.4)
        self.ctrl.stopself()
        self.door.sendMsg("doorClosed")

        self.carousel.sendMsg("threeStack")
        self.goToLocationC()
        self.alignDropOff(28)
        currTime = time.time()
        while time.time() < currTime + 1.0:
            self.ctrl.goFoward(0.25)
        self.ctrl.stopBot()
        self.door.sendMsg("doorClosed")
        self.ctrl.stopBot()

    def initBotVariables(self):
        self.initServos()
        #self.initYaw = self.realSense.getCurrYaw()

  
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
    
    print("starting program")
    print("Turn on motors")
    #time.sleep(3)
    
    bot.initBotVariables()

    #bot.initServos()
    #bot.milestone3()
    #print("initYaw: ", bot.initYaw)
    bot.initServos()

    bot.alignDropOff2(66)
    print("Ready to drop")
    bot.ctrl.stopBot()
    
    '''
    while True:
        
        print(bot.pedestal_tracker.make_prediction())
    '''
    
   
            
            
   
    