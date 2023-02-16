import cv2 as cv
from math import atan2, cos, sin, sqrt, pi
import numpy as np
import sys

from picamera.array import PiRGBArray
from picamera import PiCamera

# Python script to demonstrate working of PCA
# To be used for pedestal orientatation 
# detection for alignment of robotic arm.
# *** Works best when objects are on dark background ***

# Modified from: https://automaticaddison.com/how-to-determine-the-orientation-of-an-object-using-opencv/

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


def main():
    # Initialize the Raspberry Pi camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30
    raw_capture = PiRGBArray(camera, size=(640, 480))

    # Warm up the camera
    print("Warming up camera...")
    for _ in range(30):
        camera.capture(raw_capture, format="bgr", use_video_port=True)
        raw_capture.truncate(0)

    print("Camera ready")

    # Create a window to display the images
    cv.namedWindow("Pose Detection", cv.WINDOW_NORMAL)

    # Loop over the frames from the camera
    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        # Get the raw numpy array representing the image
        img = frame.array

        # Downsample img
        img = cv.resize(img, (0, 0), fx=0.6, fy=0.6)

        # Convert image to grayscale for contour detection
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Convert image to binary
        _, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)

        # Find all the contours in the thresholded image (binary image)
        # Countours in this scenario are a series of continous points
        # surrounding an area having uniform color or intensity.
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

            # Find the orientation of each shape
            getOrientation(c, img)

        # Display the image
        cv.imshow("Pose Detection", img)

        # Wait for a key press
        key = cv.waitKey(1) & 0xFF

        # Clear the stream in preparation for the next frame
        raw_capture.truncate(0)

        # If the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

    # Release the camera and close the window
    camera.close()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()