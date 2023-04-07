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
import cv2
import pyrealsense2 as rs
import numpy as np
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, logUsage
import jetson_utils_python
from math import sqrt, degrees, pi, atan2
import threading
import time

class RealSense:
    def __init__(self):
        #We will create a Robot command object here:
        
        self.screen_width = 640
        self.screen_height = 480
        self.frame_center = self.screen_width/2
        self.threshold = 200 
        self.detectionData = [0,0,0]

        self.yawOffset = 0.0
        self.startTime = time.time()
        
        #self.doTimer = False
        self.doAngles = True
        

        #This is for calculating yaw offsets. We are going to need to increment time in a thread

        parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, 
                                 epilog=detectNet.Usage() + videoSource.Usage() + videoOutput.Usage() + logUsage())

        parser.add_argument("input_URI", type=str, default="/dev/video2", nargs='?', help="URI of the input stream")
        parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
        parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
        parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
        parser.add_argument("--threshold", type=float, default=0.85, help="minimum detection threshold to use") 

        is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

        try:
            self.args = parser.parse_known_args()[0]
        except:
            #print("")
            parser.print_help()
            sys.exit(0)
        
        #print(self.args)

        # create video sources and outputs
        self.pipeline = rs.pipeline()
        config = rs.config()
        #Changed these from 640x480 to 1280x480
        ##config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)

        # Start the pipeline
        self.pipeline.start(config)

        ##self.detections = []

        #This for Yaw calculation thread
        self.rpy = [0.0, 0.0, 0.0]
        self.first = True
        self.alpha = 0.98
        self.totalgyroangleY = 0

        
        # create video sources and outputs
        # input = videoSource(args.input_URI, argv=sys.argv)
        ##self.output = videoOutput("display://0")

        # load the object detection network
        # self.net = detectNet(self.args.network, sys.argv, self.args.threshold)

        # note: to hard-code the paths to load a model, the following API can be used:
        '''
        self.net = detectNet(model="/home/mdelab/jetson-inference//build/aarch64/bin/ssd-mobilenet-02262023.onnx", labels="/home/mdelab/jetson-inference/build/aarch64/bin/labels.txt",
                 input_blob="input_0", output_cvg="scores", output_bbox="boxes",
                 threshold=self.args.threshold)
        '''
        
        #thread = threading.Thread(target=self.objDectect)
        #thread.start()
        print("Starting Cameera")
        self.angles = threading.Thread(target=self.angleCalculation)
        self.angles.start()

        
        #self.timer = threading.Thread(target=self.timePassed)
        #timePassed.start()

    
    def getYawOffset(self):
        #Want to track time passed for yaw offset calculation
        #Will use a boolean to init this first time we come into this fucntion
        #EveryTimer we enter this target funciton we will make sure doTimer is true do that we can
        #Correction factor is basically the slope of the error. So all we are doing is subtracting a line with the error
        #Slope from the calculated yaw angle to stabailize it
        timePassed = time.time() - self.startTime
        correctionFactor = 0.131232
        
        '''
        To tune this let the below while loop run for some time and then plot the data in YawCorrection.txt
        Data has time and yaw offset
        After plotting add a trend line and get the slope of the line
        We want the plot to be as flat as possible, so add whatever slot you see to the correction factor
        '''
        #noise = 2*np.random.normal(0,0.00001)
        noise=0
        self.yawOffset = (correctionFactor*timePassed - noise)

        return self.yawOffset
            
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

    def getObjDetect(self,i = None):
        if i is not None:
            return self.detectionData[i]
        else:
            return self.detectionData

    def objDectect(self):
        
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
            scores = {}
            for d in self.detections:
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
            self.detectionData = []
            
            if len(scores) > 0:
                max_score = max(scores.keys())
                target = scores[max_score]
                self.detectionData = [int(target[0]), int(target[1])]
                
            else:
                self.detectionData = [0,0,0]


            # render the image
            self.output.Render(img)

            # update the title bar
            self.output.SetStatus("{:s} | Network {:.0f} FPS".format(self.args.network, self.net.GetNetworkFPS()))
            # print out performance info
            #self.net.PrintProfilerTimes()

            # exit on input/output EOS
            if not self.output.IsStreaming():
                break


    """
    This script is based on the code from the following sources:
    https://github.com/IntelRealSense/librealsense/tree/master/examples/motion
    https://github.com/IntelRealSense/librealsense/issues/4391



    Code explanation from ChatGPT:

    A complementary filter is a type of filter that combines information from two or more sources to produce a 
    more accurate estimate of a signal or a system's behavior. The term "complementary" refers to the fact that 
    the filter combines information from two or more sources that are complementary to each other, meaning that 
    they provide information about different aspects of the signal or system.

    A complementary filter is often used to fuse information from multiple sensors, such as an accelerometer 
    and a gyroscope, to estimate the orientation of a device, for example. The accelerometer provides information 
    about the linear acceleration of the device, while the gyroscope provides information about the angular 
    velocity. By combining these two sources of information, a complementary filter can produce a more accurate 
    estimate of the orientation of the device.

    The basic idea behind a complementary filter is to use a low-pass filter to smooth out the noisy accelerometer 
    measurements and a high-pass filter to capture the fast changes in orientation provided by the gyroscope. 
    The two filters are then combined using a simple weighted sum to produce the final estimate of the orientation.

    Complementary filters are widely used in many applications, including inertial navigation systems, 
    robotics, and aerospace engineering, where accurate estimation of signals or system behavior is critical. 
    They are simple to implement and can be easily adapted to different applications by adjusting 
    the filter parameters.

    """
    def angleCalculation(self):
        while self.doAngles:
           
            f = self.pipeline.wait_for_frames()
            
            #gather IMU data
            accel = f[0].as_motion_frame().get_motion_data()
            gyro = f[1].as_motion_frame().get_motion_data()

            ts = f.get_timestamp()

            #calculation for the first frame
            if (self.first):
                
                self.first = False
                last_ts_gyro = ts

                # accelerometer calculation
                accel_angle_z = degrees(atan2(accel.y, accel.z))
                accel_angle_x = degrees(atan2(accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)))
                accel_angle_y = degrees(pi)
                continue


            #calculation for the second frame onwards

            # gyrometer calculations
            dt_gyro = (ts - last_ts_gyro) / 1000
            last_ts_gyro = ts

            gyro_angle_x = gyro.x * dt_gyro
            gyro_angle_y = gyro.y * dt_gyro
            gyro_angle_z = gyro.z * dt_gyro

            dangleX = gyro_angle_x * 57.2958
            dangleY = gyro_angle_y * 57.2958
            dangleZ = gyro_angle_z * 57.2958

            totalgyroangleX = accel_angle_x + dangleX
            # totalgyroangleY = accel_angle_y + dangleY
            self.totalgyroangleY = accel_angle_y + dangleY + self.totalgyroangleY
            totalgyroangleZ = accel_angle_z + dangleZ

            #accelerometer calculation
            accel_angle_z = degrees(atan2(accel.y, accel.z))
            accel_angle_x = degrees(atan2(accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)))
            # accel_angle_y = math.degrees(math.pi)
            accel_angle_y = 0

            #combining gyrometer and accelerometer angles
            combinedangleX = totalgyroangleX * self.alpha + accel_angle_x * (1-self.alpha)
            combinedangleZ = totalgyroangleZ * self.alpha + accel_angle_z * (1-self.alpha)
            combinedangleY = self.totalgyroangleY
        


            self.rpy = [combinedangleX, combinedangleY - 180 - self.getYawOffset(), combinedangleZ]
            #print(self.rpy[1])
    
    def getCurrYaw(self):
        return round(self.rpy[1],2)


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

