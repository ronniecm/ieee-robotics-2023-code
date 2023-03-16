# Create class that uses the machine learnining models trained in the other files to classify pedestals from
# a cv2 video stream. An object will be able to make a function call to classify a frame and return
# information about the pedestals in the frame.

import cv2
import torch
import torch.nn as nn
import torch.optim as optim

import numpy as np
import torch.nn.functional as F

from ImageClassifierNets import LightweightCNN

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
        self.camera = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink", cv2.CAP_GSTREAMER)

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
        return pred.item()
    

    def make_prediction(self):
        """
        Prediction function for the class, use this to classify a frame.
        """


        # Get the frame from the camera
        ret, frame = self.camera.read()

        # Check if the frame was captured successfully
        if not ret:
            print("Failed to capture frame from camera")
            return -1

        # Classify the frame
        pred = self.__classify_frame(frame)

        # Return the prediction
        return pred