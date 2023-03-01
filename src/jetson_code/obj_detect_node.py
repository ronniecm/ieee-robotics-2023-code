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
from std_msgs.msg import Int32MultiArray
#setting up the ros node
pub = rospy.Publisher('obj_detect', Int32MultiArray, queue_size=10)
rospy.init_node('object_detect_node', anonymous=True)
rate =rospy.Rate(10)#!/usr/bin/env python3
prevDectect = False
actions = []


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
net = detectNet(model="/home/mdelab/jetson-inference//build/aarch64/bin/ssd-mobilenet-02262023.onnx", labels="/home/mdelab/jetson-inference/build/aarch64/bin/labels.txt",
                 input_blob="input_0", output_cvg="scores", output_bbox="boxes",
                 threshold=args.threshold)

import cv2

# Calculate score for each detection based on distance and class
def getScore(distance, class_id):
    score = 0

    pedestal_weight = 25
    duck_weight = 2

    # Handle case for when distance is 0
    if distance == 0:
        return 0

    if class_id == 1:
        score += pedestal_weight * np.sqrt(distance)
    elif class_id == 2:
        score = duck_weight * np.sqrt(distance)
    else:
        score = 0

    return score


def homeing(max_score_tuple):

    #Now we extract tuple date    
    class_id, score, dist, center_x = max_score_tuple
    
    print('Aligning with ', class_id)
    
    #This is a goal state
    if dist < 0.17:
        print("MADE IT TO OBJECT!!!")
        i = 0
        while(i <1):
            i+=1
            pub.publish(0)

        j = 0
        while(j<2500):
            j+=1
            #we will stop for little so that we can remove the object/load caresol
            print("REMOVE PEDESTAL")
            pub.publish(4)
        #Now we will go back to the home position
        #Write something here to keep track of actions taken to get to this point
        #We will need to write a function to go back to home position
        
        #Now reverse the actions that it took to get to the object
        print("REVERSING")
        #print(actions)
        #for action in actions:
        #    pub.publish(action)
        #    rate.sleep()
        #    print(actions)

        #Now that we have returned to our starting postion we can clear the actions that it 
        #took to get to the object so that we can record actions to get to the next object
        
        #actions.clear()
        #Now that we chave gone through all the actions we need to tell it to stop
        pub.publish(4)
        return

    screen_width = 640
    frame_center = screen_width/2
    threshold = 200 
    #Now we need to check if we are in a 100 pixel range of the center of the frame
    if center_x > frame_center + threshold/2 or center_x < frame_center - threshold/2:
        
        #We need to check  if center bounding box is on left or right of screen
        if center_x >= frame_center + threshold/2:
            #Move right if bounding box is on right side of screen
            pub.publish(2)
            #will append the oposite so that we can go back to the starting position
            #actions.append(3)
            
        elif center_x <= frame_center - threshold/2:
            #Move right if bound box is on left side of screen
            pub.publish(3)
            #actions.append(2)
        else:
            pass
    
    elif center_x < frame_center + threshold/2 and center_x > frame_center - threshold/2 and dist > 0.16:
        pub.publish(0)
        #actions.append(6)
        
    
    #We've reach end goal we can go tow    # img = input.Capture()

def shutdown():
    pub.publish(4)


def filter_detections(detections):
		"""
		Filter out "double" detections of the same object within the same frame.
		"""
		# For each detection, compare how similar the center pixel is to all other center pixels
		# and filter out the ones that are too similar, taking care to not compare a detection with itself, as well
		# as to not make a comparison twice (i.e. i vs j and j vs i)
		for i in range(len(detections)):
			for j in range(i + 1, len(detections)):
				if i != j:
					# Check that j in range
					if j >= len(detections):
						break

					# Get the center of each detection
					center_i = detections[i].Center
					center_j = detections[j].Center

					# Calculate the distance between the two centers
					dist = np.sqrt((center_i[0] - center_j[0])**2 + (center_i[1] - center_j[1])**2)
					# If the distance is less than the threshold, remove the detection with the lower score
					threshold = 10
					if dist < threshold:
						
						if detections[i].Confidence < detections[j].Confidence:
							detections.pop(i)
						else:
							detections.pop(j)
                                                        
# process frames until the user exits
while True:
    # Real Sense frame data: Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
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
    detections = net.Detect(img, overlay=args.overlay)

    # print the detections
    print("detected {:d} objects in image".format(len(detections)))

    #Might need this somewhere else in the code
    scores = {}
    filter_detections(detections)
    for d in detections:
        score = 0
        class_id = int(d.ClassID)
        center = d.Center
        depth_value = depth_frame.get_distance(int(center[0]),int(center[1])) * 100
        if depth_value > 0:
            ground_distance = np.sqrt(depth_value*depth_value - 13*13)
            if ground_distance > 7:
                dist = np.sqrt(ground_distance * ground_distance - 49)
                
        score = getScore(center[0], class_id)
        scores[score] = (class_id, center[0])
    msg = Int32MultiArray()
    if len(scores) > 0:
        max_score = max(scores.keys())
        target = scores[max_score]
        msg.data = [int(target[0]), int(target[1])]
    else:
        msg.data = [0,0,0]     
        
    pub.publish(msg)
    # render the image
    
    output.Render(img)

    # update the title bar
    #output.SetStatus("{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS()))
	# print out performance info
	#net.PrintProfilerTimes()

    # exit on input/output EOS
    if not output.IsStreaming():
        break
