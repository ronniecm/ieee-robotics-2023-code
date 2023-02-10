#!/usr/bin/env python3

import pyrealsense2 as rs

# Initialize the RealSense D435i
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.pose)
pipeline.start(config)

# Retrieve the orientation information
frames = pipeline.wait_for_frames()
pose_frame = frames.get_pose_frame()
if pose_frame:
    data = pose_frame.get_pose_data()
    quaternion = data.rotation
    yaw = math.atan2(2.0 * (quaternion.z * quaternion.w),
                     1.0 - 2.0 * (quaternion.z * quaternion.z))
    print("Yaw: {:.2f} degrees".format(math.degrees(yaw)))

# Stop the pipeline
pipeline.stop()