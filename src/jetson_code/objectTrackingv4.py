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
    import pyrealsense2 as rs


    from jetson_inference import detectNet
    from jetson_utils import videoSource, videoOutput, logUsage
    import jetson_utils_python

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float64

#This class will serve as the direct communication from jetson to arduino
#Using this class will allow us to talk to the arduino

class RobotCommand:
    def __init__(self, robot_name, node_name, command_topic, msg_type, queue_size):

        try:
            rospy.init_node("%s_%s" %(robot_name, node_name), anonymous=True)
            self.pub = rospy.Publisher("%s/%s" %(robot_name,command_topic), msg_type, queue_size =10)

            rospy.Subscriber('%s/ultraFront' %robot_name, Float64, self.ultraFront)
            rospy.Subscriber('%s/ultraRight' %robot_name, Float64, self.ultraRight)
            rospy.Subscriber('%s/ultraBack' %robot_name, Float64, self.ultraBack)
            rospy.Subscriber('%s/ultraLeft' %robot_name, Float64, self.ultraLeft)

            print("ROS SETUP OKAY")

        except:
            print("SOMETHING ROS RELATED FUCKED UP")
            pass
            #print("Could no set up ROS topic: %s/%s" %(robot_name, command_topic))

           

        #Using .pub() to publish messages to robot
        self.robot_name = robot_name
        self.msg_type = msg_type
        self.command_topic = command_topic
        self.rate = rospy.Rate(10)
        self.prevDectect = False
        self.reversedActions = []
        

    
    '''
    ROS Callback functions
    '''

    def ultraFront(self, msg):
        print(msg.data, "GOT READING!!!!")
        pass
    def ultraRight(self, msg):
        pass
    def ultraBack(self, msg):
        pass
    def ultraLeft(self, msg):
        pass

        #Now we will set up the subscribers for the ultrasonic sensors


        #This should be all that we need to run once to make sure that everything is setup okay
        #Now we can make different commands for the robot
    
    def buildMsg(self, msg_type, msg):
        #Put the foward kinematics stuff here
        pass

    def goFoward(self):
        self.pub.publish(0)
        pass
    def goBackwards(self):
        self.pub.publish(6)
        pass
    def goRight(self):
        self.pub.publish(2)
        pass
    def goLeft(self):
        self.pub.publish(3)
        pass
    def rotateLeft(self):
        pass
    def rotateRight(self):
        pass
    def stopBot(self):
        self.pub.publish(4)



#This will line it self up with object, right now works in x component and then y 
#But will use foward kinematics soon 

class RealSense:
    def __init__(self,botObject):
        #We will create a Robot command object here:
        self.bot = botObject

        self.screen_width = screen_width
        self.screen_height = screen_height
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
        parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

        is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

        try:
	        self.args = parser.parse_known_args()[0]
        except:
	        print("")
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
    
        print('Aligning with ', class_id)
    
        #This is a goal state
        if dist < 0.17:
            print("MADE IT TO OBJECT!!!")
            i = 0
            while(i <1):
                i+=1
                self.bot.goFoward()

            j = 0
            while(j<2500):
                j+=1
            #we will stop for little so that we can remove the object/load caresol
                print("REMOVE PEDESTAL")
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
    def centerToRight(self, center_x):
        if center_x >= self.frame_center + self.threshold/2:
            return True
        else:
            return False

    def centerToLeft(self, center_x):
        if center_x <= self.frame_center - self.threshold/2:
            return True
        else:
            return False

    def getScore(distance, class_id):
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
        if len(self.detections ) == 0:
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
            #print("Frame shape", color_imageRotated.shape)

            # Convert the color numpy array to a CUDA image for inference
            img = jetson_utils_python.cudaFromNumpy(color_image)

            # capture the next image
            # img = input.Capture()

            # detect objects in the image (with overlay)
            self.detections = self.net.Detect(img, overlay=self.args.overlay)

            # print the detections
            print("detected {:d} objects in image".format(len(detections)))

            #Might need this somewhere else in the code
            center_pixel_dist = depth_frame.get_distance(int(320),int(240))

            
            if self.detections.isEmpty():
 
                self.bot.stopBot()
                
                #elif center_pixel_dist >= 0.5:
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

                for d in detections:
                    class_id = d.ClassID
                    center_x, center_y = d.Center
                    print("Detection center",d.Center)
                    dist = depth_frame.get_distance(int(center_x),int(center_y))
                    #If we are getting distance then there is something in frame
                    #but we are not guarenteed that it is accurate
                    if dist > 0:
                        score = getScore(dist, class_id)
                        #print("CurrentScore" , score)
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
                    #print("There are: ", len(scores), " scores")
                    #print("All Scores: ", scores)
                    max_score = max(scores.keys())
                    #print("Max Scores: ", max_score)
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

if __name__ == "__main__":
    #Takes in camera dimensions
    bot = RobotCommand("bot","talker","cmd_vel", Int8, queue_size = 10)
    while True:
        bot.stopBot()
        break
        
    #camera = RealSense(bot)
    #camera.run()


      
#TODO
'''
Look into how to access IMU data:
Follow link: https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md

Need to make a function that subscribes to Arduino ultrasonic sensors
    They need a function in the void loop that publishes ultra sonic readingsr

'''


    






    

    


        