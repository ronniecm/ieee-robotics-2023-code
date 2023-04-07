import cv2 as cv
from math import atan2, cos, sin, sqrt, pi
import numpy as np
import sys
import torch
from ImageClassifierNets import LightweightCNN

# Python script to demonstrate working of PCA
# To be used for pedestal orientatation 
# detection for alignment of robotic arm.
# *** Works best when objects are on dark background ***

# Modified from: https://automaticaddison.com/how-to-determine-the-orientation-of-an-object-using-opencv/

# Modified by: Jhonny Velasquez
# Last Modified: 02-02-2023

# Create moving average filter class for smoothing
class MovingAverageFilter:
    def __init__(self, size):
        self.size = size
        self.data = []

    def add(self, value):
        self.data.append(value)
        if len(self.data) > self.size:
            self.data.pop(0)

    def get(self):
        return sum(self.data) / len(self.data)


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
    cv.line(img, (int(p[0]), int(p[1])),
            (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)

    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv.line(img, (int(p[0]), int(p[1])),
            (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)

    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv.line(img, (int(p[0]), int(p[1])),
            (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)
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
    mean, eigenvectors, eigenvalues = cv.PCACompute2(data_pts, mean)

    # Store the center of the object
    cntr = (int(mean[0, 0]), int(mean[0, 1]))
    # [pca]

    # [visualization]
    # Draw the principal components
    cv.circle(img, cntr, 3, (255, 0, 255), 2)
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
    textbox = cv.rectangle(
        img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255, 255, 255), -1)
    cv.putText(img, label, (cntr[0], cntr[1]),
               cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv.LINE_AA)

    return angle

def img_2_tensor(img):
    # Resize image to 160x120 using INTER_AREA interpolation
    img = cv.resize(img, (160, 120), interpolation=cv.INTER_AREA)

    # Convert image to RGB and normalize
    img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB).astype(np.float32)
    cv.normalize(img_rgb, img_rgb, 0, 1, cv.NORM_MINMAX)

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
    cam = cv.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv.CAP_GSTREAMER)
    # Warm up the camera
    if not cam.isOpened():
        print("Could not open camera")
        exit(0)

    print("Camera ready")

    # Create a window to display the images
    cv.namedWindow("Pose Detection", cv.WINDOW_NORMAL)

    # Create moving average objects for smoothing out angle measurements
    angle_avg = MovingAverageFilter(20)

    # Initialize model
    print("Initializing model...")

    # Load the model
    model = LightweightCNN()
    device = 'cpu'
    model_path = '/home/mdelab/ieee-robotics-2023-code/src/jetson_code/pedestal_classification/lightweight_net_color_orientation_2023-04-06_17-17-38.pth'
    model.load_state_dict(torch.load(
        model_path, map_location=torch.device(device)))
    model.eval()

    # Loop over the frames from the camera
    while True:

        # Print seperator between frames
        print("--------------------------------------------------")

        # Get the raw numpy array representing the image
        ret, img = cam.read()

        if not ret:
            print("Failed to capture frame from camera")
            break

        # Convert image to tensor for classification
        img_tensor = img_2_tensor(img)
        with torch.no_grad():
            _, pred = torch.max(model(img_tensor), 1)
        
        print_prediction(pred.item())

        # Now that we have the prediction, find contours on the correct color frame
        img = cv.resize(img, (0, 0), fx=0.1, fy=0.1) # Downsample img

        frame = None
        
        # Check if frame is green or red, white is already grayscale
        if pred == 0 or pred == 1:
            frame = img[:, :, 1]

        elif pred == 2 or pred == 3:
            frame = img[:, :, 2]

        else:
            frame = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Default value for frame

        # Convert image to binary
        _, bw = cv.threshold(frame, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)

        # Find all the contours in the thresholded image (binary image)
        contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

        # Filter out contours that are too small or too large
        min_countour_area = 1000
        max_countour_area = 100000

        # Loop over all the contours
        for i, c in enumerate(contours):

            # Calculate the area of each contour
            area = cv.contourArea(c)

            # Ignore contours that are too small or too large
            # Could be used to filter out noise
            if area < min_countour_area or area > max_countour_area:
                continue

            # Draw each contour only for visualisation purposes
            cv.drawContours(img, contours, i, (0, 0, 255), 2)

            # Find the orientation
            angle = getOrientation(c, img)

            # Convert to degrees
            deg_angle = -int(np.rad2deg(angle)) - 90

            # Apply moving average filter
            angle_avg.add(deg_angle)
            smoothed = angle_avg.get()

            print("Angle: ", smoothed)

        # Display the image
        cv.imshow("Pose Detection", img)

        # Wait for a key press
        key = cv.waitKey(1) & 0xFF

        # If the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

    # Release the camera and close the window
    cam.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()