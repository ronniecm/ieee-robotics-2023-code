import cv2

def main():
    # Open the video stream from the CSI-connected Raspberry Pi Camera v2
    cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink", cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Error: Unable to open video stream.")
        return

    while True:
        # Read a frame from the video stream
        ret, frame = cap.read()
        if not ret:
            break

        # Display the frame
        cv2.imshow("Raspberry Pi Camera Stream", frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Release the video stream and destroy the OpenCV window
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
