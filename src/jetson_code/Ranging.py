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
from std_msgs.msg import Float32MultiArray


class Ranging:
    def __init__(self, robot_name, node_name):
        self.sim = False

        if robot_name == 'sim':
            self.sim = True
            ultra_msg_type = Range
            
        else:
            ultra_msg_type = Float32
                

        #rospy.Subscriber('%s/ultra0' %robot_name, Float32, self.ultra0)
        #rospy.Subscriber('%s/ultra1' %robot_name, Float32, self.ultra1)
        rospy.Subscriber('%s/ultra2' %robot_name, ultra_msg_type, self.ultra2)
        rospy.Subscriber('%s/ultra3' %robot_name, ultra_msg_type, self.ultra3)
        rospy.Subscriber('%s/ultra4' %robot_name, ultra_msg_type, self.ultra4)
        rospy.Subscriber('%s/ultra5' %robot_name, ultra_msg_type, self.ultra5)
        rospy.Subscriber('%s/ultra6' %robot_name, ultra_msg_type, self.ultra6)
        rospy.Subscriber('%s/ultra7' %robot_name, ultra_msg_type, self.ultra7)

        rospy.Subscriber('%s/tofSensors' %robot_name, Float32MultiArray, self.tofSensors)

        '''
        Each face of the robot has a list assigned to it. You can access the sensors on that face through a list
        That list is will be self.rng.get<FACE>
        So if I wanted to get sensor readings from the sensor on the front left I would call self.rng.getFront(0)
        
        0 index corresponds to a left sensor on a face and 1 corresponds to the right sensor on the face

        Make sure this indexing is consistant througout ALL error correction calculations otherwise there were will unintentional erros
        This is how indexing should work such that function work the same for all sides and no need to repeat any calculations
        If this not working make sure to check the call ultrasonic callback functions to make sure the distances are being indexed correcly I 
        may have messed up !!
                                                -Front-
                                            _0_______1_
                                        1 |           | 0
                                            |           |
                                            |           |
                                        0 |           | 1
                                            |___________|
                                                1       0   
        '''

    
        self.ultraFront = [0.0, 0.0]
        self.ultraRight = [0.0, 0.0]
        self.ultraBack = [0.0, 0.0]
        self.ultraLeft = [0.0, 0.0]

        self.tofDist = [0.0,  0.0]

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
                        0           1       (TOF)

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

    def tofSensors(self, msg):
        self.tofDist[0] = msg.data[0]
        self.tofDist[1] = msg.data[1]

    def getTofSensors(self, i = None):
        if i is not None:
            return self.tofDist[i]
        else:
            return self.tofDist

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