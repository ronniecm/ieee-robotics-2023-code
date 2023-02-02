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

import sys
import argparse

# import pyrealsense2 as rs

import numpy as np
import cv2
import pyrealsense2 as rs


from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, logUsage
import jetson_utils_python

import rospy
from std_msgs.msg import Int8

#setting up the ros node
pub = rospy.Publisher('chatter', Int8, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate =rospy.Rate(15)#!/usr/bin/env python3
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

import sys
import argparse

# import pyrealsense2 as rs

import numpy as np
import cv2
import pyrealsense2 as rs


from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, logUsage
import jetson_utils_python

import rospy
from std_msgs.msg import Int8

#setting up the ros node
pub = rospy.Publisher('chatter', Int8, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate =rospy.Rate(15)

import time


# parse the command line
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
	args = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# create video sources and outputs
pipeline = rs.pipeline()
config = rs.config()
#Changed these from 640x480 to 1280x480
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline
pipeline.start(config)

# create video sources and outputs
# input = videoSource(args.input_URI, argv=sys.argv)
output = videoOutput("display://0")

# load the object detection network
net = detectNet(args.network, sys.argv, args.threshold)

# note: to hard-code the paths to load a model, the following API can be used:
#
net = detectNet(model="/home/mdelab/Downloads/ssd-mobilenet-finetuned-v3.onnx", labels="/home/mdelab/Downloads/labels.txt",
                 input_blob="input_0", output_cvg="scores", output_bbox="boxes",
                 threshold=args.threshold)

import cv2

# Calculate score for each detection based on distance and class
def getScore(distance, class_id):
    score = 0

    pedestal_weight = 10
    duck_weight = 2

    # Handle case for when distance is 0
    if distance == 0:
        return 0

    if class_id < 4 and class_id >= 1:
        score += pedestal_weight/np.sqrt(distance)
    elif class_id == 4:
        score = duck_weight/np.sqrt(distance)
    else:
        score = 0

    return score


def homeing(max_score_tuple):

    #Now we extract tuple date
    print('In homeing')
    

    class_id, score, dist, center_x = max_score_tuple
    screen_width = 640

    frame_center = screen_width/2
    threshold = 100
    #Now we need to check if we are in a 100 pixel range of the center of the frame
    if center_x > frame_center + threshold/2 or center_x < frame_center - threshold/2:
        #We need to move left
        pub.publish(3)
        pass
        
        #We need to check  if center bounding box is on left or right of screen
        if center_x > frame_center + threshold/2:
            #Move right if bounding box is on right side of screen
            pub.publish(2)
            
        elif center_x < frame_center - threshold/2:
            #Move right if bound box is on left side of screen
            pub.publish(3)
        else:
            pass

    elif center_x < frame_center + threshold/2 and center_x > frame_center - threshold/2:
        pub.publish(0)
    #We've reach end goal we can go towards it now

def shutdown():
    pub.publish(4)

rospy.on_shutdown(shutdown)

# process frames until the user exits
while True:
    # Real Sense frame data: Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Convert the color frame to a numpy array
    color_image = np.asanyarray(color_frame.get_data())
    color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

    # Convert the color numpy array to a CUDA image for inference
    img = jetson_utils_python.cudaFromNumpy(color_image)

    # capture the next image
    # img = input.Capture()

    # detect objects in the image (with overlay)
    detections = net.Detect(img, overlay=args.overlay)

    # print the detections
    print("detected {:d} objects in image".format(len(detections)))

    if len(detections) == 0:
        pub.publish(1)

    else:
        #Will keep use scores for key in dict
        scores = {}
        inFrame = []
        score = 0
        #This will be for case that score is zero

        for d in detections:
            class_id = d.ClassID
            center_x, center_y = d.Center
            dist = depth_frame.get_distance(int(center_x),int(center_y))
            #If we are getting distance then there is something in frame
            #but we are not guarenteed that it is accurate
            if dist > 0:
                score = getScore(dist, class_id)
                print("CurrentScore" , score)
                #Score will always be greater than zero here
                info = (class_id, score, dist, center_x)
                scores[score] = info
        #Return the hiest score in the dictionary after looping through all detections and taking there scores

        
        #now that we have the highest score we will home in on info corresponing to highest score

        #For the the case that score dict is never updated we will stop for now
        if len(scores) == 0:
            pub.publish(4)
        else:
            # Return the greatest key in the scores dictionary
            print("All Scores: ", scores)
            max_score = max(scores.keys())
            print("Max Scores: ", max_score)
            homeing(scores[max_score])
    
    # render the image
    output.Render(img)

    # update the title bar
    #output.SetStatus("{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS()))
	# print out performance info
	#net.PrintProfilerTimes()

    # exit on input/output EOS
    if not output.IsStreaming():
        break

# parse the command line
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
	args = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# create video sources and outputs
pipeline = rs.pipeline()
config = rs.config()
#Changed these from 640x480 to 1280x480
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline
pipeline.start(config)

# create video sources and outputs
# input = videoSource(args.input_URI, argv=sys.argv)
output = videoOutput("display://0")

# load the object detection network
net = detectNet(args.network, sys.argv, args.threshold)

# note: to hard-code the paths to load a model, the following API can be used:
#
net = detectNet(model="/home/mdelab/Downloads/ssd-mobilenet-finetuned-v3.onnx", labels="/home/mdelab/Downloads/labels.txt",
                 input_blob="input_0", output_cvg="scores", output_bbox="boxes",
                 threshold=args.threshold)

import cv2

# Calculate score for each detection based on distance and class
def getScore(distance, class_id):
    score = 0

    pedestal_weight = 10
    duck_weight = 2

    # Handle case for when distance is 0
    if distance == 0:
        return 0

    if class_id < 4 and class_id >= 1:
        score += pedestal_weight/np.sqrt(distance)
    elif class_id == 4:
        score = duck_weight/np.sqrt(distance)
    else:
        score = 0

    return score


def homeing(max_score_tuple):

    #Now we extract tuple date
    print('In homeing')
    

    class_id, score, dist, center_x = max_score_tuple
    screen_width = 640

    frame_center = screen_width/2
    threshold = 100
    #Now we need to check if we are in a 100 pixel range of the center of the frame
    if center_x > frame_center + threshold/2 or center_x < frame_center - threshold/2:
        #We need to move left
        pub.publish(3)
        pass
        
        #We need to check  if center bounding box is on left or right of screen
        if center_x > frame_center + threshold/2:
            #Move right if bounding box is on right side of screen
            pub.publish(2)
            
        elif center_x < frame_center - threshold/2:
            #Move right if bound box is on left side of screen
            pub.publish(3)
        else:
            pass

    elif center_x < frame_center + threshold/2 and center_x > frame_center - threshold/2:
        pub.publish(0)
    #We've reach end goal we can go towards it now

def shutdown():
    pub.publish(4)

rospy.on_shutdown(shutdown)

# process frames until the user exits
while True:
    # Real Sense frame data: Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Convert the color frame to a numpy array
    color_image = np.asanyarray(color_frame.get_data())
    color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

    # Convert the color numpy array to a CUDA image for inference
    img = jetson_utils_python.cudaFromNumpy(color_image)

    # capture the next image
    # img = input.Capture()

    # detect objects in the image (with overlay)
    detections = net.Detect(img, overlay=args.overlay)

    # print the detections
    print("detected {:d} objects in image".format(len(detections)))

    if len(detections) == 0:
        pub.publish(1)

    else:
        #Will keep use scores for key in dict
        scores = {}
        inFrame = []
        score = 0
        #This will be for case that score is zero

        for d in detections:
            class_id = d.ClassID
            center_x, center_y = d.Center
            dist = depth_frame.get_distance(int(center_x),int(center_y))
            #If we are getting distance then there is something in frame
            #but we are not guarenteed that it is accurate
            if dist > 0:
                score = getScore(dist, class_id)
                print("CurrentScore" , score)
                #Score will always be greater than zero here
                info = (class_id, score, dist, center_x)
                scores[score] = info
        #Return the hiest score in the dictionary after looping through all detections and taking there scores

        
        #now that we have the highest score we will home in on info corresponing to highest score

        #For the the case that score dict is never updated we will stop for now
        if len(scores) == 0:
            pub.publish(4)
        else:
            # Return the greatest key in the scores dictionary
            print("All Scores: ", scores)
            max_score = max(scores.keys())
            print("Max Scores: ", max_score)
            homeing(scores[max_score])
    
    # render the image
    output.Render(img)

    # update the title bar
    #output.SetStatus("{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS()))
	# print out performance info
	#net.PrintProfilerTimes()

    # exit on input/output EOS
    if not output.IsStreaming():
        break
