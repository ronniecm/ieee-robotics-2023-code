#!/usr/bin/env python3

import cv2
from math import atan2, cos, sin, sqrt, pi
import numpy as np

# Import necessary pytorch libraries
import torch
import torch.nn as nn
import torch.nn.functional as F

import time

# Python script to demonstrate working of PCA
# To be used for pedestal orientatation 
# detection for alignment of robotic arm.
# *** Works best when objects are on dark background ***

# Modified from: https://automaticaddison.com/how-to-determine-the-orientation-of-an-object-using-opencv2/

# Modified by: Jhonny Velasquez
# Last Modified: 02-02-2023

def drawAxis(img, p_, q_, color, scale):
    '''
    Function to draw axis for an object
    '''
    
    p = list(p_)
    q = list(q_)

    # [visualization1]
    angle = atan2(p[1] - q[1], p[0] - q[0])  # angle in radians
    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) +
                      (p[0] - q[0]) * (p[0] - q[0]))

    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * cos(angle)
    q[1] = p[1] - scale * hypotenuse * sin(angle)
    cv2.line(img, (int(p[0]), int(p[1])),
            (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)

    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])),
            (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)

    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])),
            (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
    # [visualization1]


def getOrientation(pts, img):
    '''
    Function to calculate the orientation of an object
    '''

    # [pca]
    # Construct a buffer used by the pca analysis
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i, 0] = pts[i, 0, 0]
        data_pts[i, 1] = pts[i, 0, 1]

    # Perform PCA analysis
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)

    # Store the center of the object
    cntr = (int(mean[0, 0]), int(mean[0, 1]))
    # [pca]

    # [visualization]
    # Draw the principal components
    cv2.circle(img, cntr, 3, (255, 0, 255), 2)
    p1 = (cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0],
          cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0])
    p2 = (cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0],
          cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0])
    drawAxis(img, cntr, p1, (255, 255, 0), 1)
    drawAxis(img, cntr, p2, (0, 0, 255), 5)

    # orientation in radians
    angle = atan2(eigenvectors[0, 1], eigenvectors[0, 0])
    # [visualization]

    # Label with the rotation angle
    label = "  Rotation Angle: " + \
        str(-int(np.rad2deg(angle)) - 90) + " degrees"
    textbox = cv2.rectangle(
        img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255, 255, 255), -1)
    cv2.putText(img, label, (cntr[0], cntr[1]),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    return angle

# Defines the light weight CNN model that will be used for classification
class LightweightCNN(nn.Module):
    def __init__(self):
        super(LightweightCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, kernel_size=3, stride=2, padding=1)
        self.maxpool1 = nn.MaxPool2d(kernel_size=4, stride=2)

        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2, padding=1)
        self.maxpool2 = nn.MaxPool2d(kernel_size=4, stride=2)
        
        self.conv3 = nn.Conv2d(32, 64, kernel_size=5, stride=2, padding=1)
        self.maxpool3 = nn.MaxPool2d(kernel_size=2, stride=2)

        self.fc1 = nn.Linear(64, 32)
        self.dropout = nn.Dropout(p=0.5)
        self.fc2 = nn.Linear(32, 6)

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = self.maxpool1(x)
        x = F.relu(self.conv2(x))
        x = self.maxpool2(x)
        x = F.relu(self.conv3(x))
        x = self.maxpool3(x)

        # Reshape for fully connected layer
        x = torch.flatten(x, 1)
        
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = self.fc2(x)
        
        return x
    
def img_2_tensor(img):
    # Resize image to 160x120 using INTER_AREA interpolation
    img = cv2.resize(img, (160, 120), interpolation=cv2.INTER_AREA)

    # Convert image to RGB and normalize
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
    cv2.normalize(img_rgb, img_rgb, 0, 1, cv2.NORM_MINMAX)

    # Swap the dimensions and convert to tensor
    img_tensor = torch.as_tensor(img_rgb).permute(2, 0, 1)

    return img_tensor.unsqueeze(0)

def print_prediction(prediction):
    if prediction == 0:
        print("Green & Fallen")
    elif prediction == 1:
        print("Green & Standing")
    elif prediction == 2:
        print("Red & Fallen")
    elif prediction == 3:
        print("Red & Standing")
    elif prediction == 4:
        print("White & Fallen")
    elif prediction == 5:
        print("White & Standing")
    else:
        print("Error")


def main():
    # Initialize the camera object
    #cam = cv2.VideoCapture('udp://127.0.0.1:5000')
    #cam = cv2.VideoCapture('/dev/video0')
    #cam = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink", cv2.CAP_GSTREAMER)
    
    cam = cv2.VideoCapture(1)
    # Check if the camera was opened successfully
    if not cam.isOpened():
        print("Could not open camera")
        exit(0)

    # Create a model instance and load the weights to it
    model = LightweightCNN()
    model.load_state_dict(torch.load('/Users/jvelasquez/Virginia_Tech/Spring_2023/ECE_4806/ieee-robotics-2023-code/src/jetson_code/pedestal_classification/lightweight_net_color_orientation_v3.pth'))
    model.eval() # Needed to set the model to evaluation mode

    # used to record the time when we processed last frame
    prev_frame_time = 0
  
    # used to record the time at which we processed current frame
    new_frame_time = 0

    while True:
        # Capture a frame from the camera
        ret, img = cam.read()

        # Was the frame captured successfully?
        if not ret:
            print("Failed to capture frame from camera")
            break
        
        # Convert the image to a tensor and pass it through the model
        img_tensor = img_2_tensor(img)
        with torch.no_grad():
            _, pred = torch.max(model(img_tensor), 1)

        # Print the prediction
        print_prediction(pred)

        # Calculating the fps
        new_frame_time = time.time()
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time
        fps = str(int(fps))
        cv2.putText(img, fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv2.LINE_AA)

        # Display the image in a window
        cv2.imshow('Camera Feed', img)

        # Press q to quit
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the camera and destroy all windows
    cam.release()
    cv2.destroyAllWindows()

def gst_str():
    # Default GST pipeline for the Jetson Nano camera
    return 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)YUY2, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'


if __name__ == "__main__":
    main()