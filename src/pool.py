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

from asv.msg import CGAction, CGGoal, CGResult, CGFeedback, Constraint, ControllerAction, ControllerGoal, ControllerResult, ControllerFeedback, Vector, Info_msg

import sys
import os
sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/')

import planner.border_planner as pl 

import numpy as np 


class Pool:
    ''' Init node '''
    def __init__(self):
        # INIT NODE
        rospy.init_node('pool')
        self.rate = rospy.Rate(0.5)
        self.position1 = None
        self.position2 = None
        self.position3 = None


        self.points1 = [-5.0,5.0,-5.0,5.0]
        self.points2 = [-5.0,5.0,-5.0,5.0]
        self.points3 = [-5.0,5.0,-5.0,5.0]

        self.radius = 3


        self.bordP1 = pl.BorderPlanner(limits = self.points1, starting_slope = 0.7, radius = self.radius)
        self.bordP2 = pl.BorderPlanner(limits = self.points2, starting_slope = -0.7, radius = self.radius)
        self.bordP3 = pl.BorderPlanner(limits = self.points3, starting_slope = 0.3, radius = self.radius)

        self.local_info1 = rospy.Subscriber('/asv1/vehicle/info', Info_msg, self.position1_callback)
        self.local_info2 = rospy.Subscriber('/asv2/vehicle/info', Info_msg, self.position2_callback)
        self.local_info3 = rospy.Subscriber('/asv3/vehicle/info', Info_msg, self.position3_callback)

        self.ref_pub1 = rospy.Publisher('/asv1/cg/reference', Vector, queue_size=10)
        self.ref_pub2 = rospy.Publisher('/asv2/cg/reference', Vector, queue_size=10)
        self.ref_pub3 = rospy.Publisher('/asv3/cg/reference', Vector, queue_size=10)


    def position1_callback(self, msg):
        tmp = msg.x 
        #self.position = np.array([tmp[0],tmp[1]])
        self.position1 = [tmp[0], tmp[1]]
        #self.position = np.reshape(self.position, (2,1))

    def position2_callback(self, msg):
        tmp = msg.x 
        #self.position = np.array([tmp[0],tmp[1]])
        self.position2 = [tmp[0], tmp[1]]
        #self.position = np.reshape(self.position, (2,1))

    def position3_callback(self, msg):
        tmp = msg.x 
        #self.position = np.array([tmp[0],tmp[1]])
        self.position3 = [tmp[0], tmp[1]]
        #self.position = np.reshape(self.position, (2,1))

        
    def loop(self):
        #rospy.sleep(1.0)
        while not rospy.is_shutdown():
            if(self.position1 is None):
                position = rospy.wait_for_message('/asv1/vehicle/info', Info_msg)
                position = position.x
                self.position1 = [position[0], position[1]]         
                #self.position = np.reshape(self.position, (2,1))
            if(self.position2 is None):
                position = rospy.wait_for_message('/asv2/vehicle/info', Info_msg)
                position = position.x
                self.position2 = [position[0], position[1]]   
            if(self.position3 is None):
                position = rospy.wait_for_message('/asv3/vehicle/info', Info_msg)
                position = position.x
                self.position3 = [position[0], position[1]]
            #print(self.position)

            cg_r1 = self.bordP1.compute_reference(self.position1)
            cg_r2 = self.bordP2.compute_reference(self.position2)
            cg_r3 = self.bordP3.compute_reference(self.position3)
            #print(cg_r)

            if((cg_r1 is not None)):
                #print(cg_r)
                self.ref_pub1.publish(cg_r1)

            if((cg_r2 is not None)):
                self.ref_pub2.publish(cg_r2)

            if((cg_r3 is not None)):    
                self.ref_pub3.publish(cg_r3) 

            #print(cg_r)
            self.rate.sleep()
            #return 


        





if __name__ == '__main__':

    simu_pool_exp = Pool()
    simu_pool_exp.loop()
    


