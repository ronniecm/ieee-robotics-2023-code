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

# Modified by: Jhonny Velasquez
# Date: 02-01-2023

import sys
import argparse

import pyrealsense2 as rs

import numpy as np

from jetson_utils import videoSource, videoOutput, logUsage

from jetson_inference import detectNet
import jetson_utils_python


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

# initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline
pipeline.start(config)

output = videoOutput("display://0")
	
# load the object detection network
net = detectNet(args.network, sys.argv, args.threshold)

# note: to hard-code the paths to load a model, the following API can be used:
#
net = detectNet(model="/home/mdelab/Downloads/ssd-mobilenet-finetuned-v3.onnx", labels="/home/mdelab/Downloads/labels.txt", 
                 input_blob="input_0", output_cvg="scores", output_bbox="boxes", 
                 threshold=args.threshold)

# Import openCV here so that it doesn't conflict with the jetson-inference imports
import cv2

# process frames until the user exits
while True:

	# Real Sense frame data: Wait for a coherent pair of frames: depth and color

    # Wait for the next set of frames from the pipeline
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
    output.Render(img)
    # print the detections
    print("detected {:d} objects in image".format(len(detections)))

    for detection in detections:
        print(detection)
        x, y = detection.Center
	
	    # trying to add visual feedback to the image    
        # cv2.circle(color_frame, (x,y), 8, (0,0,255), -1)

        print(detection.Center)
        depth_value = depth_frame.get_distance(int(x),int(y))

        # Print the depth value
        print("Depth value (cm):", depth_value * 100)

	    # render the image
        output.Render(img)

	    # update the title bar
        output.SetStatus("{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS()))

	    # print out performance info
        net.PrintProfilerTimes()

	    # exit on input/output EOS
        if not output.IsStreaming():
            break
        #if not input.IsStreaming() or not output.IsStreaming():
        #    break
