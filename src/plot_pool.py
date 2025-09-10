#  GNU License (C) 2025 LaSa, DIMES, Univerity of Calabria

# This program is developed for LaSa, DIMES, Univerity of Calabria. 
# Its copy, use, redistribution or modification is prohibited, or requires
# you to ask for permission. All authorized modifications made to 
# the software are subject to the same conditions as the original software.
# This program is provided as is: WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# For a copy of the complete license please contact elqemmah.ay@dimes.unical.it.

# Minimal notes added; program logic unchanged.

#! /usr/bin/env python3
import rospy
import time
import actionlib
import matplotlib.pyplot as plt

from asv.msg import CGAction, CGGoal, CGResult, CGFeedback, Constraint, ControllerAction, ControllerGoal, ControllerResult, ControllerFeedback, Vector, Info_msg

import sys
import os
sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/')

import planner.line_planner as pl 

import numpy as np 


class Plot:
    ''' Init node '''
    def __init__(self):
        # INIT NODE
        rospy.init_node('plotting_node')
        self.rate = rospy.Rate(10)
        self.position1 = None
        self.x1 = list()
        self.y1 = list()

        self.position2 = None
        self.x2 = list()
        self.y2 = list()

        self.position3 = None
        self.x3 = list()
        self.y3 = list()

        self.g = list()

    def loop(self):
        #rospy.sleep(1.0)
        plt.axis([-7.0, 7.0, -7.0, 7.0])
        p_left_bl = np.ones((11,1))*(-5.0)
        p_right_bl = np.ones((11,1))*(5.0)
        p_bottom_bl = np.ones((11,1))*(-5.0)
        p_top_bl = np.ones((11,1))*(5.0)

        plt.plot(p_left_bl, np.arange(-5.0,6.0,1))
        plt.plot(p_right_bl, np.arange(-5.0,6.0,1))
        plt.plot(np.arange(-5.0,6.0,1), p_bottom_bl)
        plt.plot(np.arange(-5.0,6.0,1), p_top_bl)

        while not rospy.is_shutdown():
            if(self.position1 is None):
                position = rospy.wait_for_message('/asv1/vehicle/info', Info_msg)
                position = position.x
                self.position1 = [position[0], position[1]] 
                self.x1.append(position[0])
                self.y1.append(position[1])  

            if(self.position2 is None):
                position = rospy.wait_for_message('/asv2/vehicle/info', Info_msg)
                position = position.x
                self.position2 = [position[0], position[1]] 
                self.x2.append(position[0])
                self.y2.append(position[1])  

            if(self.position3 is None):
                position = rospy.wait_for_message('/asv3/vehicle/info', Info_msg)
                position = position.x
                self.position3 = [position[0], position[1]] 
                self.x3.append(position[0])
                self.y3.append(position[1])  

            position = rospy.wait_for_message('/asv1/vehicle/info', Info_msg)
            position = position.x  
            self.x1.append(position[0])
            self.y1.append(position[1])

            position = rospy.wait_for_message('/asv2/vehicle/info', Info_msg)
            position = position.x  
            self.x2.append(position[0])
            self.y2.append(position[1])

            position = rospy.wait_for_message('/asv3/vehicle/info', Info_msg)
            position = position.x  
            self.x3.append(position[0])
            self.y3.append(position[1])

            plt.plot(self.x1, self.y1, 'r') 
            plt.plot(self.x2, self.y2, 'b')
            plt.plot(self.x3, self.y3, 'y')
            plt.pause(0.05)
            #if(len(self.x) == len(self.y)):   
            #    plt.plot(self.x, self.y) 
            #    plt.pause(0.05)
            #print(self.x)
            #plt.show()
            self.rate.sleep()
            #return 


        





if __name__ == '__main__':

    plot_pos = Plot()
    plot_pos.loop()
    


