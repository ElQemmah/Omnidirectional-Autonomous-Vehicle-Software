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

from asv.msg import CGAction, CGGoal, CGResult, CGFeedback, Constraint, ControllerAction, ControllerGoal, ControllerResult, ControllerFeedback, Vector

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
        self.position = None
        self.x = list()
        self.y = list()
        self.g = list()
        #self.local_info = rospy.Subscriber('/asv1/vehicleinfo', Vector, self.position_callback)

    """
    def position_callback(self, msg):
        
        tmp = msg.data 
        #self.position = np.array([tmp[0],tmp[1]])
        self.position = [tmp[0], tmp[1]]
        self.x.append(tmp[0])
        self.y.append(tmp[1])
        #self.position = np.reshape(self.position, (2,1))
    """
        
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
            if(self.position is None):
                position = rospy.wait_for_message('/asv1/vehicleinfo', Vector)
                position = position.data
                self.position = [position[0], position[1]] 
                self.x.append(position[0])
                self.y.append(position[1])  
            position = rospy.wait_for_message('/asv1/vehicleinfo', Vector)
            position = position.data  
            self.x.append(position[0])
            self.y.append(position[1])
            plt.plot(self.x, self.y) 
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
    


