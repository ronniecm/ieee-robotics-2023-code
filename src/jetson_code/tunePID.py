#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
from PyQt5.QtCore import Qt
import sys

class JointControl(QWidget):
    def __init__(self):
        super(JointControl, self).__init__()
        pi = 3.14
        rospy.init_node('joint_control', anonymous=True)
        # Create a Publisher object for the /p1_arm_controller/command topic
        self.pub = rospy.Publisher('/bot/PID', Float64MultiArray, queue_size=10)


        # Create four slider widgets and label widgets
        self.sliders = [QSlider(Qt.Horizontal) for i in range(3)]
        self.labels = [QLabel('P'), QLabel('I'), QLabel('D')]

        # Set the minimum and maximum values for the sliders as floats
        min_value = 0
        max_value = 20.0

        for slider in self.sliders:
            slider.setMinimum(0)
            slider.setMaximum(int(max_value * 100))
            slider.setSingleStep(1)
        
        # Create a button widget to publish the position command
        self.button = QPushButton('Publish PID Command')
        self.button.clicked.connect(self.publish_position)

        # Create a layout to organize the widgets
        layout = QVBoxLayout()
        for slider, label in zip(self.sliders, self.labels):
            layout.addWidget(slider)
            layout.addWidget(label)
        layout.addWidget(self.button)

        # Set the layout for the main widget
        self.setLayout(layout)

        # Connect the slider signals to the update label slots
        for i, slider in enumerate(self.sliders):
            slider.valueChanged.connect(lambda value, i=i: self.update_label(i, value))

    def update_label(self, index, value):
        # Update the label text with the current position value
        self.labels[index].setText('Position {}: {:.2f}'.format(index+1, value / 100.0))

    def publish_position(self):
        # Get the current position values from the sliders
        positions = [slider.value() / 100.0 for slider in self.sliders]

        # Create a Float64MultiArray message and populate it with the position values
        msg = Float64MultiArray()
        msg.data = positions

        # Publish the message to the /p1_arm_controller/command topic
        self.pub.publish(msg)


if __name__ == '__main__':
    # Initialize the ROS node and PyQt5 application
    app = QApplication(sys.argv)
    
    # Create the joint control widget and show it
    widget = JointControl()
    widget.show()
    
    # Run the PyQt5 application
    sys.exit(app.exec_())

