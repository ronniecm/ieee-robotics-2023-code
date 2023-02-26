#!/usr/bin/env python3

#==========================================
# Title:  Robot Ranging Class
# Author: Juan Suquilanda
# Date:   26 February 2023
#==========================================


'''
Class for all ranging sensors used for robot. ROS callback functions give user 
real-time  access to ranging data
'''


import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32


class Ranging:
    def __init__(self, robot_name, node_name):
        self.sim = False

        if robot_name == 'sim':
            self.sim = True
            ultra_msg_type = Range
            
        else:
            ultra_msg_type = Float32
        

        rospy.init_node("%s_%s" %(robot_name, node_name), anonymous=True)
        

        #rospy.Subscriber('%s/ultra0' %robot_name, Float32, self.ultra0)
        #rospy.Subscriber('%s/ultra1' %robot_name, Float32, self.ultra1)
        rospy.Subscriber('%s/ultra2' %robot_name, ultra_msg_type, self.ultra2)
        rospy.Subscriber('%s/ultra3' %robot_name, ultra_msg_type, self.ultra3)
        rospy.Subscriber('%s/ultra4' %robot_name, ultra_msg_type, self.ultra4)
        rospy.Subscriber('%s/ultra5' %robot_name, ultra_msg_type, self.ultra5)
        rospy.Subscriber('%s/ultra6' %robot_name, ultra_msg_type, self.ultra6)
        rospy.Subscriber('%s/ultra7' %robot_name, ultra_msg_type, self.ultra7)

    
        self.ultraFront = [0.0, 0.0]
        self.ultraRight = [0.0, 0.0]
        self.ultraBack = [0.0, 0.0]
        self.ultraLeft = [0.0, 0.0]

    def ultra2(self, msg):
        #print("Top Right Reading: %s" %msg.range)
        if self.sim:
            self.ultraRight[0] = msg.range * 100
        else:
            self.ultraRight[0] = msg.data
        

    def ultra3(self, msg):
        ##print("Bottom Right Reading: %s" %msg.range)
        if self.sim:
            self.ultraRight[1] = msg.range * 100
        else:
            self.ultraRight[1] = msg.data

        
    def ultra4(self, msg):
        ##print("Back Right Reading: %s" %msg.range)
        #if abs(self.ultraBack[0] - msg.range) < 3:
        if self.sim:
            self.ultraBack[0] = msg.range * 100
        else:
            self.ultraBack[0] = msg.data
        ##print("List now contains: " , self.ultraBack)

    def ultra5(self, msg):
        ##print("Back Left Reading: %s" %msg.range)
        #if abs(self.ultraBack[1] - msg.range) < 3:
        if self.sim:
            self.ultraBack[1] = msg.range * 100
        else:
            self.ultraBack[1] = msg.data

        ##print("List now contains: " , self.ultraBack)
        
    def ultra6(self, msg):
        ##print("Bottom Left Reading: %s" %msg.range)
        #if abs(self.ultraLeft[0] - msg.range) < 3:
        if self.sim:
            self.ultraLeft[0] = msg.range * 100
        else:
            self.ultraLeft[0] = msg.data
            
        ##print("List now contains: " , self.ultraLeft)

    def ultra7(self, msg):
        ##print("Top Left Reading: %s" %msg.range)
        #if abs(self.ultraLeft[1] - msg.range) < 3:
        if self.sim:
            self.ultraLeft[1] = msg.range * 100
        else:
            self.ultraLeft[1] = msg.data


            '''

            -------UlTRA SONIC NAMING DIAGRAM----------

                            FRONT

                     ___0___________1___
                    |                   |
                  7 |                   | 2
                    |                   |
            LEFT    |                   |   RIGHT
                    |                   |
                    |                   | 
                  6 |                   | 3
                    |___________________|
                        5           4

                            BACK
            '''
    def getFront(self,i = None):
        if i is not None:
            return self.ultraFront[i]
        else:
            return self.ultraFront

    def getRight(self,i = None):
        if i is not None:
            return self.ultraRight[i]
        else:
            return self.ultraRight

    def getBack(self,i = None):
        if i is not None:
            return self.ultraBack[i]
        else:
            return self.ultraBack
        
    def getLeft(self,i = None):
        if i is not None:
            return self.ultraLeft[i]
        else:
            return self.ultraLeft