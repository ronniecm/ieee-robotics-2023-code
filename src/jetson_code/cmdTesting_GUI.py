#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
from PyQt5.QtCore import Qt
import sys

class ArmControl(QWidget):
    def __init__(self):
        super(ArmControl, self).__init__()
        pi = 3.14
        rospy.init_node('joint_control', anonymous=True)
        # Create a Publisher object for the /scara_arm_controller/command topic
        self.pub_gripper_rotate = rospy.Publisher('/bot/gripperRotate_cmd', Int16, queue_size=10)
        self.pub_gripper_clamp = rospy.Publisher('/bot/gripperClamp_cmd', Int16, queue_size=10)
        self.pub_door = rospy.Publisher('/bot/door_cmd', Int16, queue_size=10)
        self.pub_arm = rospy.Publisher('/bot/arm_cmd', Int16, queue_size=10)
        self.pub_wrist = rospy.Publisher('/bot/wrist_cmd', Int16, queue_size=10)
        self.pub_paddle = rospy.Publisher('/bot/paddle_cmd', Int16, queue_size=10)
        self.pub_lifting = rospy.Publisher('/bot/lifting_cmd', Int16, queue_size=10)
        self.pub_carousel = rospy.Publisher('/bot/carousel_cmd', Int16, queue_size=10)


        # Create four slider widgets and label widgets
        self.sliders = [QSlider(Qt.Horizontal) for i in range(7)]
        self.labels = [QLabel('Gripper Rotate'), QLabel('Gripper Clamp'), QLabel('Door'), QLabel('Arm'), QLabel('Wrist'), QLabel('Paddle'), QLabel('Lifting'), QLabel('Carousel')]
        
        # Set the minimum and maximum values for the sliders as floats
        min_value = 0
        max_value = 180

        for slider in self.sliders:
            slider.setMinimum(int(min_value * 100))
            slider.setMaximum(int(max_value * 100))
            slider.setSingleStep(1)
    
        
        # Create a button widget to publish the carousel command
        self.button1 = QPushButton('Publish Carousel Cmd')
        self.button1.clicked.connect(self.carouselCommand)

        # Create a button widget to publish the position command
        self.button2 = QPushButton('Publish Arm Cmds')
        self.button2.clicked.connect(self.publishArmCommands)

        # Create a layout to organize the widgets
        layout = QVBoxLayout()
        for slider, label in zip(self.sliders, self.labels):
            layout.addWidget(label)
            layout.addWidget(slider)
        layout.addWidget(self.button1)
        layout.addWidget(self.button2)

        # Set the layout for the main widget
        self.setLayout(layout)

        
        # Connect the slider signals to the update label slots
        for i, slider in enumerate(self.sliders):
            slider.valueChanged.connect(lambda value, i=i: self.update_label(i, value))

    def update_label(self, index, value):
        # Update the label text with the current position value
        labels = {0: 'Gripper Rotate', 1: 'Gripper Clamp', 2: 'Door', 3: 'Arm', 4: 'Wrist', 5: 'Paddle', 6: 'Lifting', 7: 'Carousel'}
        self.labels[index].setText('%s : %.2f' %(labels[index], value / 100.0))

    def publishArmCommands(self):
        # Get the current position values from the sliders
        cmds = [slider.value() / 100.0 for slider in self.sliders]
        #print(cmds)

        self.pub_gripper_rotate.publish(int(cmds[0]))
        self.pub_gripper_clamp.publish(int(cmds[1]))
        self.pub_door.publish(int(cmds[2]))
        self.pub_arm.publish(int(cmds[3]))
        self.pub_wrist.publish(int(cmds[4]))
        self.pub_paddle.publish(int(cmds[5]))
        self.pub_lifting.publish(int(cmds[6]))

    def carouselCommand(self):
        self.pub_carousel.publish(int(1))


if __name__ == '__main__':
    # Initialize the ROS node and PyQt5 application
    app = QApplication(sys.argv)
    
    # Create the joint control widget and show it
    widget = ArmControl()
    widget.show()
    
    # Run the PyQt5 application
    sys.exit(app.exec_())