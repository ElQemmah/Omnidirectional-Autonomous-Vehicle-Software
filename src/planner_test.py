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

import sys
import os
sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/')

import planner.line_planner as pl 

import numpy as np 


class Planner_test:
    ''' Init node '''
    def __init__(self):
        # INIT NODE
        rospy.init_node('planner_test_experiment')
        self.rate = rospy.Rate(0.5)
        self.position = None
        self.points = [10.0,10.0]
        self.radius = 5
        self.lineP = pl.LinePlanner(points = self.points, radius = self.radius)

        self.local_info = rospy.Subscriber('/asv1/vehicleinfo', Vector, self.position_callback)
        self.ref_pub = rospy.Publisher('/asv1/cg/reference', Vector, queue_size=10)


    def position_callback(self, msg):
        tmp = msg.data 
        #self.position = np.array([tmp[0],tmp[1]])
        self.position = [tmp[0], tmp[1]]
        #self.position = np.reshape(self.position, (2,1))

        
    def loop(self):
        #rospy.sleep(1.0)
        while not rospy.is_shutdown():
            if(self.position is None):
                position = rospy.wait_for_message('/asv1/vehicleinfo', Vector)
                position = position.data
                self.position = [position[0], position[1]]         
                #self.position = np.reshape(self.position, (2,1))

            #print(self.position)
            cg_r = self.lineP.compute_reference(self.position)
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

    simu_plan = Planner_test()
    simu_plan.loop()
    


