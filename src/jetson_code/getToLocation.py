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
#


# Modified by: Ronnie Mohapatra
# Modified by Juan Suquilanda
# Modified by: Jhonny Velasquez

onJetson = False

if onJetson:

    import sys
    import argparse
    import time

    # import pyrealsense2 as rs

    import numpy as np
    import cv2
    #import pyrealsense2 as rs


    from jetson_inference import detectNet
    from jetson_utils import videoSource, videoOutput, logUsage
    import jetson_utils_python
import time
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

#This class will serve as the direct communication from jetson to arduino
#Using this class will allow us to talk to the arduino

class RobotCommand:
    def __init__(self, robot_name, node_name, command_topic, msg_type, queue_size):

        try:
            rospy.init_node("%s_%s" %(robot_name, node_name), anonymous=True)
            self.pub = rospy.Publisher("%s/%s" %(robot_name,command_topic), msg_type, queue_size =10)

            #rospy.Subscriber('%s/ultra0' %robot_name, Float32, self.ultra0)
            #rospy.Subscriber('%s/ultra1' %robot_name, Float32, self.ultra1)
            rospy.Subscriber('%s/ultra2' %robot_name, Float32, self.ultra2)
            rospy.Subscriber('%s/ultra3' %robot_name, Float32, self.ultra3)
            rospy.Subscriber('%s/ultra4' %robot_name, Float32, self.ultra4)
            rospy.Subscriber('%s/ultra5' %robot_name, Float32, self.ultra5)
            rospy.Subscriber('%s/ultra6' %robot_name, Float32, self.ultra6)
            rospy.Subscriber('%s/ultra7' %robot_name, Float32, self.ultra7)
            rospy.Subscriber('/obj_detect', Int8, self.obj_detect)
            rospy.Subscriber('/yaw', Float32, self.yaw)

            '''

            -------UlTRA SONIC NAMING DIAGRAM----------

                            FRONT

                     ___0___________1___
                    |                   |
                  7 |                   | 2
                    |                   |
            LEFT    |                   |   RIGHT
                    |                   |
                    |                   | 
                  6 |                   | 3
                    |___________________|
                        5           4

                            BACK
            '''

            #print("ROS SETUP OKAY")

        except:
            #print("SOMETHING ROS RELATED FUCKED UP")
            pass
            ##print("Could no set up ROS topic: %s/%s" %(robot_name, command_topic))

           

        #Using .pub() to publish messages to robot
        self.robot_name = robot_name
        self.msg_type = msg_type
        self.command_topic = command_topic
        self.rate = rospy.Rate(10)
        self.prevDectect = False
        self.reversedActions = []
        
        #Initializeing ultrasonic distance measurements, This will prob go in a new CLASS
        #This also may be able to be saved as an array intead of individual
        #This where we make tuples
        
        self.ultraFront = [0.0, 0.0]
        self.ultraRight = [0.0, 0.0]
        self.ultraBack = [0.0, 0.0]
        self.ultraLeft = [0.0, 0.0]
        self.objDetect = 0
        self.currYawAngle = 0.0 - 180.0 #This will ensure our starting yaw will be 0 degees easier calculations
        self.initBoardWidth = 0.0
        self.initYaw = 0.0
        self.botWidth = 30.0
        
    
    '''
    ROS Callback functions for ultrasonics
    '''
    def yaw(self, msg):
        self.currYawAngle = msg.data
        #rint("Reading: %s" %msg.data)
    def obj_detect(self, msg):
        self.objDetect = msg.data
        #rint("Reading: %s" %msg.data)SYNC_SEC

    def ultra0(self, msg):
        ##print("Front Left Reading: %s" %msg.data)
        self.ultraFront[0] = msg.data
        ##print("List now contains: " , self.ultraFront)

    def ultra1(self, msg):
        ##print("Front Right Reading: %s" %msg.data)
        self.ultraFront[1] = msg.data
        ##print("List now contains: " , self.ultraFront)

    def ultra2(self, msg):
        #print("Top Right Reading: %s" %msg.data)
        self.ultraRight[0] = msg.data 
        '''if self.ultraRight[0] == 0.0 and abs(self.ultraRight[0] - msg.data) > 3:
            self.ultraRight[0] = msg.data 
        elif abs(self.ultraRight[0] - msg.data) <= 3:
            self.ultraRight[0] = msg.data
        else:
            self.ultraRight[0] = self.ultraRight[0]

        print("List now contains: " , self.ultraRight)'''

    def ultra3(self, msg):
        ##print("Bottom Right Reading: %s" %msg.data)
        self.ultraRight[1] = msg.data 
        '''if self.ultraRight[1] == 0.0 and abs(self.ultraRight[1] - msg.data) > 3:
            self.ultraRight[1] = msg.data 
        elif abs(self.ultraRight[1] - msg.data) <= 3:
            self.ultraRight[1] = msg.data
        else:
            self.ultraRight[1] = self.ultraRight[1]
        print("List now contains: " , self.ultraRight)'''

    def ultra4(self, msg):
        ##print("Back Right Reading: %s" %msg.data)
        #if abs(self.ultraBack[0] - msg.data) < 3:
        self.ultraBack[0] = msg.data 
        ##print("List now contains: " , self.ultraBack)

    def ultra5(self, msg):
        ##print("Back Left Reading: %s" %msg.data)
        #if abs(self.ultraBack[1] - msg.data) < 3:
        self.ultraBack[1] = msg.data 
        ##print("List now contains: " , self.ultraBack)
        
    def ultra6(self, msg):
        ##print("Bottom Left Reading: %s" %msg.data)
        #if abs(self.ultraLeft[0] - msg.data) < 3:
        self.ultraLeft[0] = msg.data 
        ##print("List now contains: " , self.ultraLeft)

    def ultra7(self, msg):
        ##print("Top Left Reading: %s" %msg.data)
        #if abs(self.ultraLeft[1] - msg.data) < 3:
        self.ultraLeft[1] = msg.data 
        ##print("List now contains: " , self.ultraLeft)


    #Now we should have update values for ALL sides in their respective list for each side
        
    
    def buildMsg(self, x, y, rot):
        msg = Twist()
        #print("Before params: ", msg)
        #print("Building msg with following params: %s,%s,%s" %(x,y,rot))
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = 0.0
        
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = rot
        #print("Message Built", msg)
        #print(msg)
        return msg
    
    #List of basic commands that can be used as geometry twist messages

    def goFoward(self):
        # Move the robot forward
        msg = self.buildMsg(1.0, 0.0, 0.0)
        self.pub.publish(msg)
        
    def goBackwards(self):
        # Move the robot backwards
        msg = self.buildMsg(-1.0, 0.0, 0.0)
        self.pub.publish(msg)
        
    def goRight(self):
        # Move the robot right
        msg = self.buildMsg(0, 1.0, 0.0)
        self.pub.publish(msg)
        
    def goLeft(self):
        # Move the robot left
        msg = self.buildMsg(0.0, -1.0, 0.0)
        self.pub.publish(msg)

    # Rotate the robot left
    def rotateLeft(self):
        msg = self.buildMsg(0.0, 0.0, -0.25)
        self.pub.publish(msg)

    def rotateRight(self):
        # Rotate the robot right
        msg = self.buildMsg(0.0, 0.0, 0.25)
        self.pub.publish(msg)
        
    def stopBot(self):
        # Stop the robot
        msg = self.buildMsg(0.0, 0.0, 0.0)
        self.pub.publish(msg)
        time.sleep(0.5)



    '''
    Each face of the robot has a list assigned to it. You can access the sensors on that face through a list
    That list is will be self.ultra<FACE>
    So if I wanted to get sensor readings from the sensor on the front left I would call self.ultraFront[0]
    
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

    def initStartingConditions(self):
        time.sleep(2)
        self.initYaw = self.currYawAngle
        self.initBoardWidth = self.ultraLeft[1] + self.botWidth + self.ultraRight[0]
        
    #Creating a function that will get robot to known statue locations labled A, B, C

    def goToLocationC(self):
        #It is important that in the begining of the wround the intial yaw value is saved
        #For this to happen out goal state will be dependent on back and right side sensors
        #Therefore we will create variables that we would expect to read with the ultrasonic sensors

        #TODO measure the actual values and plug into here
        c_x = 2.0
        c_y = 4.0

        #First we are going to make sure that the robot has the same yaw that it had in the begining
        #This will ensure that the sensors will be parallel to their opossing wall

        
        currYaw = self.currYawAngle
    
        '''
        Now we know how much we should rotate. Since a clockwise rotation increases yaw, and 
        counter clockwise decreases yaw. If the mod of of the current 
        '''
        #This means we will have to rotate right
        if currYaw < 0:
            yawOffset = (currYaw % 360) - 360
            while (self.currYawAngle < currYaw + abs(yawOffset)):
                self.rotateRight()

        elif (currYaw > 0):
            while(self.currYaw > currYaw - yawOffset):
                self.rotateLeft()

        #Now we should (within1inch(self.ultraright[1], c_y), 2)be in the same orientation we started in and can start moving in the direction we need to go
        #We will need to build a custom message since our location can be anywhere on the board

        #We'll take a moment and let values comes in
        self.stopBot()

        #Need to test which sensor from back is more reliable
        msg_x = -(self.ultraBack[0] - c_x)
        msg_y = 0.0


        #This condition checks to see which sensors are closer to wall therefore we can rely on them better
        if self.ultraLeft[0] > self.ultraRight[1]:
            msg_y = self.ultraRight[1] - c_y
        else:
            msg_y = self.initBoardWidth - self.ultraLeft[1] - c_y
        
        #Now we should have the vector we need to travel in for robot to get to location
        msg = self.buildMsg(msg_x, msg_y, 0)

        while not within1inch(self.ultraright[1], c_y, 2) or not within1inch(self.ultraright[1], c_y, 2):
            self.pub.publish(msg)

        self.stopBot()
        #Now we should be at or near location
        



#This will line it self up with object, right now works in x component and then y 
#But will use foward kinematics soon 

class RealSense:
    def __init__(self,botObject):
        #We will create a Robot command object here:
        self.bot = botObject

        self.screen_width = 640
        self.screen_height = 480
        self.frame_center = self.screen_width/2
        self.threshold = 200 

        #parse the command line
        parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, 
                                 epilog=detectNet.Usage() + videoSource.Usage() + videoOutput.Usage() + logUsage())

        parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
        parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
        parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
        parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
        parser.add_argument("--threshold", type=float, default=1.0, help="minimum detection threshold to use") 

        is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

        try:
	        self.args = parser.parse_known_args()[0]
        except:
	        #print("")
	        parser.print_help()
	        sys.exit(0)

        # create video sources and outputs
        self.pipeline = rs.pipeline()
        config = rs.config()
        #Changed these from 640x480 to 1280x480
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start the pipeline
        self.pipeline.start(config)

        # create video sources and outputs
        # input = videoSource(args.input_URI, argv=sys.argv)
        self.output = videoOutput("display://0")

        # load the object detection network
        self.net = detectNet(self.args.network, sys.argv, self.args.threshold)

        # note: to hard-code the paths to load a model, the following API can be used:
        self.net = detectNet(model="/home/mdelab/Downloads/ssd-mobilenet-finetuned-v3.onnx", labels="/home/mdelab/Downloads/labels.txt",
                 input_blob="input_0", output_cvg="scores", output_bbox="boxes",
                 threshold=self.args.threshold)

        
    def homeing(self, max_score_tuple):

        #Now we extract tuple date    
        class_id, score, dist, center_x = max_score_tuple
    
        #print('Aligning with ', class_id)
    
        #This is a goal state
        if dist < 0.17:
            #print("MADE IT TO OBJECT!!!")
            i = 0
            while(i <1):
                i+=1
                self.bot.goFoward()

            j = 0
            while(j<2500):
                j+=1
            #we will stop for little so that we can remove the object/load caresol
                #print("REMOVE PEDESTAL")
                self.bot.stopBot()

            return
            
        #Now we need to check if we are in a 100 pixel range of the center of the frame
        if not self.inCenterFrame(center_x):
        
            #We need to check  if center bounding box is on left or right of screen
            if self.centerToRight(center_x):
                 #Move right if bounding box is on right side of screen
                self.bot.goRight()
                #will append the oposite so that we can go back to the starting position
                #actions.append(3)
            
            elif self.centerToLeft(center_x):
                self.bot.goLeft()
            
    
        elif self.inCenterFrame(center_x):
            self.bot.goFoward()
            #actions.append(6)

   
    '''
    REALSENSE CLASS HElPER FUNCTIONS
    '''
    def inCenterFrame(self, center_x):
        if center_x < self.frame_center + self.threshold/2 and center_x > self.frame_center - self.threshold/2:
            return True
        else:
            return False

    def centerToLeft(self, center_x):
        if center_x <= self.frame_center - self.threshold/2:
            return True
        else:
            return False

    def getScore(self, distance, class_id):
        score = 0
        pedestal_weight = 25
        duck_weight = 2

        #Handle case for when distance is 0
        if distance == 0:
            return 0

        if class_id < 4 and class_id >= 1:
            score += pedestal_weight/np.sqrt(distance)
        elif class_id == 4:
            score = duck_weight/np.sqrt(distance)
        else:
            score = 0

        return score

    def isEmpty():
        if len(self.detections) == 0:
            return True
        else:
            return False


    '''
    This is like the main loop
    '''

    

    def run(self):
        
        while True:
            # Real Sense frame data: Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            # Convert the color frame to a numpy array
            color_image = np.asanyarray(color_frame.get_data())
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            
            # Rotate image 90 degrees counterclockwise
            #color_imageRotated = cv2.rotate(color_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            ##print("Frame shape", color_imageRotated.shape)

            # Convert the color numpy array to a CUDA image for inference
            img = jetson_utils_python.cudaFromNumpy(color_image)

            # capture the next image
            # img = input.Capture()            #For the the case that score dict is never updated we will stop for now

            # detect objects in the image (with overlay)
            self.detections = self.net.Detect(img, overlay=self.args.overlay)

            #Extracting pixel distance from far right and far left to measure alignment error
            #And then later correct in the checkWalls() function
            self.left_pixel_dist = depth_frame.get_distance(int(100),int(240))
            #print("Got left pixel distance: ", self.left_pixel_dist)
            self.right_pixel_dist = depth_frame.get_distance(int(540),int(240))
            #print("Got right pixel distance: ", self.right_pixel_dist)


            
            if len(self.detections) == 0:
                #Test to make implementing ultrasonics easier
                self.checkWalls()
                #self.bot.stopBot()
                
                #elif center_pixel_dist >= 1.0:
                    #when we detect something closeby we will stop rotating and move foward
                    #eventually we will encounter something that is close enough to home in on
                    #pub.publish(0)

            else:
                prevDectect = True
                #Will keep use scores for key in dict
                scores = {}
                inFrame = []
                score = 0
                #This will be for case that score is zero

                for d in self.detections:
                    class_id = d.ClassID
                    center_x, center_y = d.Center
                    #print("Detection center",d.Center)
                    dist = depth_frame.get_distance(int(center_x),int(center_y))
                    #If we are getting distance then there is something in frame
                    #but we are not guarenteed that it is accurate
                    if dist > 0:
                        score = self.getScore(dist, class_id)
                        ##print("CurrentScore" , score)
                        #Score will always be greater than zero here
                        #info = (class_id, score, dist, center_x, actions)
                        info = (class_id, score, dist, center_x)
                        scores[score] = info
                #Return the hiest score in the dictionary after looping through all detections and taking there scores

                
                #now that we have the highest score we will home in on info corresponing to highest score

                #For the the case that score dict is never updated we will stop for now
                if len(scores) == 0:
                    self.bot.stopBot()
                else:
                    # Return the greatest key in the scores dictionary
                    ##print("There are: ", len(scores), " scores")
                    ##print("All Scores: ", scores)
                    max_score = max(scores.keys())
                    ##print("Max Scores: ", max_score)
                    self.homeing(scores[max_score])
            
            # render the image
            self.output.Render(img)

            # update the title bar
            #output.SetStatus("{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS()))
            # print out performance info
            #net.PrintProfilerTimes()

            # exit on input/output EOS
            if not self.output.IsStreaming():
                break

# Check if n is within threshold of target
def within1inch(n, target, threshold=1):
    # Convert threshold to centimeters
    threshold = threshold * 2.54

    if n >= target - threshold and n <= target + threshold:
        return True
    else:
        return False


if __name__ == "__main__":
    #Takes in camera dimensions

    bot = RobotCommand("bot","talker","cmd_vel", Twist, queue_size = 10)
    bot.stopBot()
    #delay program for 5 seconds
    #time.sleep(5)
    #Need to wait for yaw angles to actually come in so will prob have to put in a delay somewhere in here
    bot.initStartingConditions()
    bot.goToLocationC
    print("turn on motors")



      
#TODO
'''
Look into how to access IMU data:
Follow links:
https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
https://www.youtube.com/watch?v=20htSO0z-F4

Need to make a function that subscribes to Arduino ultrasonic sensors
    They need a function in the void loop that publishes ultra sonic readingsr


'''


    






    

    


        