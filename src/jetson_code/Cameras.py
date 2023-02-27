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
#==========================================
# Title:  Robot Camera Class
# Author: Juan Suquilanda
# Date:   26 February 2023
#==========================================

import sys
import argparse
import time
import rospy
import cv2
import pyrealsense2 as rs
import numpy as np
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, logUsage
import jetson_utils_python
from math import sqrt

class RealSense:
	def __init__(self):
		#We will create a Robot command object here:
		
		self.screen_width = 640
		self.screen_height = 480
		self.frame_center = self.screen_width/2
		self.threshold = 200 

		#parse the command line
		parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
								 formatter_class=argparse.RawTextHelpFormatter, 
								 epilog=detectNet.Usage() + videoSource.Usage() + videoOutput.Usage() + logUsage())

		parser.add_argument("input_URI", type=str, default="/dev/video2", nargs='?', help="URI of the input stream")
		parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
		parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
		parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
		parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

		is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

		try:
			self.args = parser.parse_known_args()[0]
		except:
			#print("")
			parser.print_help()
			sys.exit(0)
		
		print(self.args)

		# create video sources and outputs
		self.pipeline = rs.pipeline()
		config = rs.config()
		#Changed these from 640x480 to 1280x480
		config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
		config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

		# Start the pipeline
		self.pipeline.start(config)

		self.detections = []

		# create video sources and outputs
		# input = videoSource(args.input_URI, argv=sys.argv)
		self.output = videoOutput("display://0")

		# load the object detection network
		# self.net = detectNet(self.args.network, sys.argv, self.args.threshold)

		# note: to hard-code the paths to load a model, the following API can be used:
		self.net = detectNet(model="ssd-mobilenet-02262023.onnx", labels="labels.txt",
				 input_blob="input_0", output_cvg="scores", output_bbox="boxes",
				 threshold=self.args.threshold)

	def filter_detections(self, detections):
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
				dist = sqrt((center_i[0] - center_j[0])**2 + (center_i[1] - center_j[1])**2)
				# If the distance is less than the threshold, remove the detection with the lower score
				threshold = 10
				if dist < threshold:
					
					if detections[i].Confidence < detections[j].Confidence:
						detections.pop(i)
					else:
						detections.pop(j)
		
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

			#Filter out detections that are too close together
			self.filter_detections(self.detections)
			
			if len(self.detections) == 0:
				#Test to make implementing ultrasonics easier
				pass
	  

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
				
			# render the image
			self.output.Render(img)

			# update the title bar
			self.output.SetStatus("{:s} | Network {:.0f} FPS".format(self.args.network, self.net.GetNetworkFPS()))
			# print out performance info
			self.net.PrintProfilerTimes()

			# exit on input/output EOS
			if not self.output.IsStreaming():
				break

