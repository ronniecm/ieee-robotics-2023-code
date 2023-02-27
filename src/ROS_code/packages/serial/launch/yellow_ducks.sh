#!/bin/bash

# Run Roscore in a new terminal window
gnome-terminal --tab --title="Roscore" -e "bash -c 'roscore; $SHELL'" &

# Wait for roscore to start
sleep 5

# Run Object Detection script
python3 object_detection.py /dev/video2 --headless &

sleep 20

# Run 1st node in a new terminal window
gnome-terminal --tab --title="Node 1" -e "bash -c 'rosrun rosserial_python serial_node.py __name:=node1 /dev/ttyACM0; $SHELL'" &

# Run 2nd node in a new terminal window
gnome-terminal --tab --title="Node 2" -e "bash -c 'rosrun rosserial_python serial_node.py __name:=node2 /dev/ttyACM1; $SHELL'" &

# Run yaw node
gnome-terminal --tab --title="Yaw" -e "bash -c 'python3 ~/ieee-robotics-code-2023/src/jetson_code/angle_calculation.py; $SHELL'" &

# Run Python Script
python3 ~/ieee-robotics-code-2023/src/jetson_code/objectTrackingv5.py
