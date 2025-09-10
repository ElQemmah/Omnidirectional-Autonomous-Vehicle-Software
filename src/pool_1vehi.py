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

from asv.msg import CGAction, CGGoal, CGResult, CGFeedback, Constraint, ControllerAction, ControllerGoal, ControllerResult, ControllerFeedback, Vector
from  asv.msg import  Info_msg
import sys
import os
sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/')

import planner.border_planner as pl 

import numpy as np 


class Pool_test_1:
    ''' Init node '''
    def __init__(self):
        # INIT NODE
        rospy.init_node('pool_experiment_1vehicle')
        self.rate = rospy.Rate(0.5)
        self.position = None
        self.points = [-5.0,5.0,-5.0,5.0]
        self.radius = 4
        self.bordP = pl.BorderPlanner(limits = self.points, starting_slope = 0.7, radius = self.radius)

        self.local_info = rospy.Subscriber('/asv1/vehicleinfo', Vector, self.position_callback)
        self.ref_pub = rospy.Publisher('/asv1/vehicle/reference', Vector, queue_size=10)


    def position_callback(self, msg):
        tmp = msg.data 
        #self.position = np.array([tmp[0],tmp[1]])
        self.position = [tmp[0], tmp[1]]
        #self.position = np.reshape(self.position, (2,1))

        
    def loop(self):
        #rospy.sleep(1.0)
        while not rospy.is_shutdown():
            if(self.position is None):
                position = rospy.wait_for_message('/asv1/vehicle/info', Info_msg)
                position = position.x
                self.position = [position[0], position[1]]         
                #self.position = np.reshape(self.position, (2,1))

            #print(self.position)
            cg_r = self.bordP.compute_reference(self.position)
            #print(cg_r)

            if((cg_r is not None)):

                #print(cg_r)
                #cg_r = cg_r[0].append(cg_r[1])
                print(cg_r)
                self.ref_pub.publish(cg_r)
            #print(cg_r)
            self.rate.sleep()
            #return 


        





if __name__ == '__main__':

    simu_plan = Pool_test_1()
    simu_plan.loop()
    


