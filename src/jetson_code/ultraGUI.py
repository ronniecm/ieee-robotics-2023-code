#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
from PyQt5.QtCore import Qt
import sys

class UltraGUI(QWidget):
    def __init__(self):
        super(UltraGUI, self).__init__()
        self.value =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.init_node('ultraGUI', anonymous=True)
        robot_name = "bot"
        ultra_msg_type = Float32
        self.labels = [QLabel('ultra2'), QLabel('ultra3'), QLabel('ultra4'), QLabel('ultra5'), QLabel('ultra6'), QLabel('ultra7')]
        rospy.Subscriber('%s/ultra2' %robot_name, ultra_msg_type, self.ultra2)
        rospy.Subscriber('%s/ultra3' %robot_name, ultra_msg_type, self.ultra3)
        rospy.Subscriber('%s/ultra4' %robot_name, ultra_msg_type, self.ultra4)
        rospy.Subscriber('%s/ultra5' %robot_name, ultra_msg_type, self.ultra5)
        rospy.Subscriber('%s/ultra6' %robot_name, ultra_msg_type, self.ultra6)
        rospy.Subscriber('%s/ultra7' %robot_name, ultra_msg_type, self.ultra7)

        rospy.Subscriber('%s/leftTOF' %robot_name, ultra_msg_type, self.tof0)
        rospy.Subscriber('%s/rightTOF' %robot_name, ultra_msg_type , self.tof1)

        layout = QVBoxLayout()

        for label in self.labels:
            layout.addWidget(label)
        

        # Set the layout for the main widget
        self.setLayout(layout)


    def ultra2(self, msg):
        #print("Top Right Reading: %s" %msg.range)
        self.value[0] = msg.data
        self.update_label(0)
        

    def ultra3(self, msg):
        ##print("Bottom Right Reading: %s" %msg.range)
        self.value[1] = msg.data
        self.update_label(1)
        
    def ultra4(self, msg):
        ##print("Back Right Reading: %s" %msg.range)
        #if abs(self.ultraBack[0] - msg.range) < 3:
        self.value[2] = msg.data
        self.update_label(2)

    def ultra5(self, msg):
        ##print("Back Left Reading: %s" %msg.range)
        #if abs(self.ultraBack[1] - msg.range) < 3:
        self.value[3] = msg.data
        self.update_label(3)
        
    def ultra6(self, msg):
        ##print("Bottom Left Reading: %s" %msg.range)
        #if abs(self.ultraLeft[0] - msg.range) < 3:
        self.value[4] = msg.data
        self.update_label(4)

    def ultra7(self, msg):
        self.value[5] = msg.data
        self.update_label(5)

    def tof0(self, msg):
        pass

    def tof1(self, msg):
        pass


    def update_label(self, index):
        # Update the label text with the current position value
        label = {0: 'ultra2', 1: 'ultra3', 2: 'ultra4', 3: 'ultra5', 4: 'ultra6', 5: 'ultra7'}
        self.labels[index].setText('%s : %.2f cm' %(label[index], self.value[index]))


if __name__ == '__main__':
    # Initialize the ROS node and PyQt5 application
    app = QApplication(sys.argv)
    
    # Create the joint control widget and show it
    widget = UltraGUI()
    widget.show()
    
    # Run the PyQt5 application
    sys.exit(app.exec_())