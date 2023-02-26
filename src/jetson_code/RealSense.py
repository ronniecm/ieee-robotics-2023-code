#!/usr/bin/env python3

#==========================================
# Title:  Robot Ranging Class
# Author: Juan Suquilanda
# Date:   26 February 2023
#==========================================

import sys
import argparse
import time
import rospy
import cv2
import pyrealsense2 as rs
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, logUsage
import jetson_utils_python


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

