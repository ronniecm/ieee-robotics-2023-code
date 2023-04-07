#!/bin/bash

roscore&

sleep 3

rosrun rosserial_python serial_node.py __name:='n1' /dev/ttyACM0&

rosrun rosserial_python serial_node.py __name:='n2' /dev/ttyACM2&

cd ~/jetson-inference/build/aarch64/bin/

python3 obj_detect_node.py /dev/video2 --headless&
