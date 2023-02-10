import pyrealsense2 as rs
from math import sqrt, degrees, pi, atan2

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



def initialize_camera():
    "Initialize the camera and return the pipeline."

    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    prof = p.start(conf)
    return p

# initialize the camera
p = initialize_camera()

# initialize variables
first = True
alpha = 0.98
totalgyroangleY = 0

try:

    # loop through the frames
    while True:
        f = p.wait_for_frames()

        #gather IMU data
        accel = f[0].as_motion_frame().get_motion_data()
        gyro = f[1].as_motion_frame().get_motion_data()

        ts = f.get_timestamp()

        #calculation for the first frame
        if (first):
            first = False
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
        totalgyroangleY = accel_angle_y + dangleY + totalgyroangleY
        totalgyroangleZ = accel_angle_z + dangleZ

        #accelerometer calculation
        accel_angle_z = degrees(atan2(accel.y, accel.z))
        accel_angle_x = degrees(atan2(accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)))
        # accel_angle_y = math.degrees(math.pi)
        accel_angle_y = 0

        #combining gyrometer and accelerometer angles
        combinedangleX = totalgyroangleX * alpha + accel_angle_x * (1-alpha)
        combinedangleZ = totalgyroangleZ * alpha + accel_angle_z * (1-alpha)
        combinedangleY = totalgyroangleY

        print("Angle -  X: " + str(round(combinedangleX,2)) + "   Y: " + str(round(combinedangleY,2)) + "   Z: " + str(round(combinedangleZ,2)))

finally:
    p.stop()