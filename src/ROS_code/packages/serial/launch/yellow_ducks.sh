#!/bin/bash

# Setting directories for teensy's
TEENSY_DIR_1=/dev/ttyACM0
TEENSY_DIR_2=/dev/ttyACM1

# Run Roscore in a new terminal window
gnome-terminal --tab --title="Roscore" -e "bash -c 'roscore; $SHELL'" &

# Wait for roscore to start
sleep 5

# Run Object Detection script
python3 object_detection.py /dev/video2 --headless &

# Wait for object detection to start
sleep 20

# Run 1st node in a new terminal window
gnome-terminal --tab --title="Node 1" -e "bash -c 'rosrun rosserial_python serial_node.py name:=node1 $TEENSY_DIR_1; $SHELL'"

# Error checking for node 1
if [ $? -ne 0 ]; then
  # Set new directory in case of an error
  TEENSY_DIR_1=/dev/ttyACM2
  gnome-terminal --tab --title="Node 1" -e "bash -c 'rosrun rosserial_python serial_node.py name:=node1 $TEENSY_DIR_1; $SHELL'"
fi

# Run 2nd node in a new terminal window
gnome-terminal --tab --title="Node 2" -e "bash -c 'rosrun rosserial_python serial_node.py name:=node2 $TEENSY_DIR_2; $SHELL'"

# Error checking for node 2
if [ $? -ne 0 ]; then
  # Set new directory in case of error
  TEENSY_DIR_2=/dev/ttyACM2
  gnome-terminal --tab --title="Node 2" -e "bash -c 'rosrun rosserial_python serial_node.py name:=node2 $TEENSY_DIR_2; $SHELL'"
fi

# Run yaw node
gnome-terminal --tab --title="Yaw" -e "bash -c 'python3 ~/ieee-robotics-code-2023/src/jetson_code/angle_calculation.py; $SHELL'" &

# Run Python Script
python3 ~/ieee-robotics-code-2023/src/jetson_code/objectTrackingv5.py
