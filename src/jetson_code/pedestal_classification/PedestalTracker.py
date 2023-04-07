# Pedestal Tracker class and helper functions
# Author: Jhonny Velasquez
# Created: 03/16/2023
# Last Modified: 03/16/2023

import cv2
import torch
import torch.nn as nn
import torch.optim as optim

import numpy as np
import torch.nn.functional as F

from math import atan2, cos, sin, sqrt, pi

from .ImageClassifierNets import LightweightCNN

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


    # orientation in radians
    angle = atan2(eigenvectors[0, 1], eigenvectors[0, 0])

    return angle


def img_2_tensor(img):
    # Resize image to 160x120 using INTER_AREA interpolation
    img = cv2.resize(img, (160, 120), interpolation=cv2.INTER_AREA)

    # Convert image to RGB and normalize
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
    cv2.normalize(img_rgb, img_rgb, 0, 1, cv2.NORM_MINMAX)

    # Swap the dimensions and convert to tensor
    img_tensor = torch.as_tensor(img_rgb).permute(2, 0, 1)

    # Add a dimension for the batch size and return
    return img_tensor.unsqueeze(0)

class PedestalTracker:
    """
    Uses a machine learning model to classify pedestals in a video stream.
    Prediction outputs correspond to the following classes:
    0: Green & Fallen
    1: Green & Standing
    2: Red & Fallen
    3: Red & Standing
    4: White & Fallen
    5: White & Standing
    """

    def __init__(self, model_path, device):
        # Set up the model
        self.model = LightweightCNN()
        self.model.load_state_dict(torch.load(model_path, map_location=device))
        self.model.eval()
        self.device = device

        # Set up the video stream
        self.camera = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv2.CAP_GSTREAMER)

        # Check if the camera was opened successfully
        if not self.camera.isOpened():
            print("Could not open camera")
            exit(0)

    def __classify_frame(self, frame):
        """
        Helper function to classify a frame. Returns the prediction.
        """

        # Convert the frame to a tensor and classify it
        frame = img_2_tensor(frame)
        frame = frame.to(self.device)
        
        with torch.no_grad():
            output = self.model(frame)

        _, pred = torch.max(output, 1)

        # Return the prediction
        print("Class: ", pred.item())
        return pred.item()
    

    def make_prediction(self, n_preds=5, n_angles=50):
        """
        Prediction function for the class, returns the estimated angle of the pedestal in degrees.
        A pedestal that is standing will return -1.

        A pedestal whose cylindrical axis is vertical in the frame is considered to be 0 degrees,
        and the angle increases as the pedestal is rotated counterclockwise.

        """
        assert(n_preds > 0)
        assert(n_angles > 0)

        preds = []
        for i in range(n_preds):
            # Get the frame from the camera
            ret, frame = self.camera.read()

            # Check if the frame was captured successfully
            if not ret:
                print("Failed to capture frame from camera")
                return -1, -1

            # Classify the frame
            pred = self.__classify_frame(frame)
            preds.append(pred)

        # Take the most common prediction
        pred = max(set(preds), key=preds.count)

        # First check if the pedestal is standing, if it is, then we shouldn't try to estimate the angle
        if pred == 1 or pred == 3 or pred == 5:
            return 0, 0

        # If the pedestal is fallen, then we can estimate the angle
        angles = []
        for i in range(n_angles):
            # Get the frame from the camera
            ret, frame = self.camera.read()

            # Check if the frame was captured successfully
            if not ret:
                print("Failed to capture frame from camera")
                return -1, -1

            # Downsample the frame for contour detection
            img = cv2.resize(frame, (0, 0), fx=0.1, fy=0.1)

            # Contour detection needs a one channel image, but for optimal results we will use either the 
            # red or green channel if the pedestal is red or green respectively. Otherwise we will convert the 
            # image to grayscale for a white pedestal.

            single_frame = None

            # Check if the pedestal is red
            if pred == 2:
                single_frame = img[:, :, 2] # Assume BGR format
            # Check if the pedestal is green
            elif pred == 0:
                single_frame = img[:, :, 1] # Assume BGR format
            # Otherwise the pedestal is white
            else:
                single_frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Convert image to binary
            _, bw = cv2.threshold(single_frame, 50, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

            # Find all the contours in the thresholded image (binary image)
            # Countours in this scenario are a series of continous points 
            # surrounding an area having uniform color or intensity.
            contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

            # Filter out contours that are too small or too large
            min_countour_area = 1000
            max_countour_area = 100000

            # Filter out contours that are too small or too large
            contours = [c for c in contours if min_countour_area < cv2.contourArea(c) < max_countour_area]
            
            # Print length of contours
            # print("Length of contours: ", len(contours))
            if len(contours) == 0:
                return 0, 0

            # Calculate the angle of the largest contour
            angle = getOrientation(contours[0], img)

            # Convert to degrees
            angle = -int(np.rad2deg(angle)) + 90

            # Only want angles in range of 0 to 180
            angle = angle % 180

            # Append the angle to the list of angles
            angles.append(angle)

        # Take the average of the angles as the final angle
        angle = int(np.mean(angles))

        # Want to return a tuple of the angle of the object and wrist angle to command
        wrist_cmd = 180 - angle


        # Return the angle of the pedestal in degrees
        return angle, wrist_cmd
    
