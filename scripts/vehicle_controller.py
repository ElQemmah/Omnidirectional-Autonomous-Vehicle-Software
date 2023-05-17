#! /usr/bin/env python3
# -*- coding: utf-8 -*-

#  Copyright (C) 2021  Vincenzo D'Angelo
#  This program is developed for Applicon srl by Vincenzo D'Angelo. 
#  Its copy, use, redistribution or modification is prohibited, or requires
#  you to ask for permission. All authorized modifications made to 
#  the software are subject to the same conditions as the original software.
#  This program is provided as is: WITHOUT ANY WARRANTY; without even the
#  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  For a copy of the complete license please contact develop@applicon.it.

import sys
import os
sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/')

import rospy
import numpy as np
import cvxpy as cp
import actionlib
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header, Float64, Int32
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from asv.msg import Vector, Matrix, ControllerAction, ControllerGoal, ControllerResult, ControllerFeedback, Info_msg
from cg.integrator import Integrator


class VehicleController:

    EPS = 0.5

    def __init__(self):
        ''' Init node '''
        # INIT NODE
        rospy.init_node('vehicle_controller_node')

        # PARAMETERS
        Tc = rospy.get_param('control/Tc', 0.1)
        self.rate = rospy.Rate(1/Tc)
        #self.__seq = 0
        #self.ref = np.array(rospy.get_param('control/reference', np.zeros((3,1))))
        self.Fb = np.array(rospy.get_param('control/feedback_gain', (np.zeros((3*6,1))).tolist()))
        self.Fb = np.array(self.Fb).reshape(3,6)
        self.Ff = np.array(rospy.get_param('control/feedforward_gain', (np.zeros((3*3,1))).tolist()))
        self.Ff = np.array(self.Ff).reshape(3,3)
        self.Cy = np.array(rospy.get_param('control/output_matrix', (np.zeros((3*6,1))).tolist()))
        self.Cy = np.array(self.Cy).reshape(3,6)

        #self.e = np.zeros((3,1))
        self.x = np.zeros((6,1))
        self.xc = np.zeros((3,1))
        self.r = np.zeros((3,1))

        self.theta = 0 
        

        xinitial = rospy.get_param('~xinitial',0.0)
        yinitial = rospy.get_param('~yinitial',0.0)
        yaw_initial = rospy.get_param('~yaw_initial',3.19) 
        #yaw_initial = float(yaw_initial) + 1.57
        self.x_initial = None
        self.y_initial = None
        self.yaw_initial = None
        if(self.yaw_initial is None or self.x_initial is None or self.y_initial is None):
            rospy.sleep(1.5)
            msg = rospy.wait_for_message('vehicle/state', Vector)
            tmp = msg.data
            x =  np.reshape(np.array((tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5])),(6,1))
            x = np.reshape(x, (6,1))
            self.x_initial = x[0]
            self.y_initial = x[1]
            self.yaw_initial = x[2]

        self.theta = self.yaw_initial  
        #yaw_initial = float(yaw_initial) + 1.57
        self.x = np.array([float(self.x_initial), float(self.y_initial),self.yaw_initial,0.0,0.0,0.0])
        self.x = np.reshape(self.x, (6,1))
        self.xc = np.array(np.dot(np.dot(np.linalg.pinv(self.Ff),self.Fb),self.x))
        self.xc = np.reshape(self.xc, (3,1))
        self.r = np.array([float(self.x_initial), float(self.y_initial),self.yaw_initial])
        self.r = np.reshape(self.r,(3,1))
        self.int = Integrator(self.xc, Tc, 0.6)
        self.op_mode = 0 #1 # Default: Classic Teleop Mode 

        # TOPICS
        topic_prefix = rospy.get_param('~topic_prefix')
        # for subscribe actions, topic_prefix is setted in launch file
        rospy.Subscriber('vehicle/state', Vector, self.state_callback)
        rospy.Subscriber('vehicle/reference', Vector, self.reference_callback)
        rospy.Subscriber('control_matrix_update', Matrix, self.update_matrix_callback)
        rospy.Subscriber('teleop/op_mode', Int32, self.op_mode_callback) # modify message type in OpMode enum
        self.tau_pub = rospy.Publisher(f'{topic_prefix}/vehicle/tau', Vector, queue_size=1)
        self.xc_pub = rospy.Publisher(f'{topic_prefix}/controller/state', Vector, queue_size=1)


        self.saturation_value = 12

        # ACTIONS
        self.server = actionlib.SimpleActionServer(f'{topic_prefix}/vehicle_controller', ControllerAction, self.controller_callback, False)
        self.server.start()
        

    def loop(self):
        ''' Loop '''
        while not rospy.is_shutdown():
            self.rate.sleep()

    
    def controller_callback(self, enable):
        ''' Compute the control law each Tc seconds:
            u_k = -Fb*x - Ff*xc  
        '''
        feedback = ControllerFeedback()
        while not self.server.is_preempt_requested():
            u = -np.dot(self.Fb, self.x) + np.dot(self.Ff, self.xc)
            if u[0] > self.saturation_value: u[0] = self.saturation_value#
            if u[0] < -self.saturation_value: u[0] = -self.saturation_value#
            if u[1] > self.saturation_value: u[1] = self.saturation_value#
            if u[1] < -self.saturation_value: u[1] = -self.saturation_value#
            if u[2] > 0.25*self.saturation_value: u[2] = 0.25*self.saturation_value#
            if u[2] < -0.25*self.saturation_value: u[2] = -0.25*self.saturation_value#
            feedback.error = ((self.r-np.dot(self.Cy, self.x)).flatten()).tolist()
            self.tau_pub.publish((u.flatten()).tolist())
            self.xc_pub.publish(((self.xc).flatten()).tolist())
            self.server.publish_feedback(feedback)
            self.rate.sleep()
            self.xc = self.int.integrate((self.r - np.dot(self.Cy, self.x)), 'trapezoidal')
        # Cancel goal request received
        result = ControllerResult()
        result.status = 'success' if np.linalg.norm(self.r-np.dot(self.Cy, self.x)) <= self.EPS else 'aborted'
        self.server.set_preempted(result,'controller stopped')

        
    def state_callback(self, msg):
        ''' Update vehicle's current state '''
        tmp = msg.data
        x =  np.reshape(np.array((tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5])),(6,1))
        x = np.reshape(x, (6,1))
        theta = x[2]
        cont_angle = self.theta//(2*np.pi)
        theta_old2 = cont_angle*(2*np.pi) + theta
        if abs(self.theta - theta_old2) > 2 and (self.theta - theta_old2) > 0:
            cont_angle += 1
        elif abs(self.theta - theta_old2) > 2 and (self.theta - theta_old2) < 0:
            cont_angle -= 1
        self.theta = cont_angle*(2*np.pi) + theta
        x[2] = self.theta
        #rospy.loginfo('%s X-Coordinate, \n %s Y-Coordinate, \n %s Theta', self.x[0], self.x[1], self.x[2])
        
        if self.op_mode > 0: # Teleop mode
            # Update reference with current position for next dynamic positioning
            self.r = self.x[0:3]
            self.xc = np.dot(np.dot(np.linalg.pinv(self.Ff),self.Fb),self.x)
        self.x = x
    

    def reference_callback(self, msg):
        ''' Update vehicle's reference '''
        if self.op_mode == 0: # Autonomous mode
            tmp = msg.data
            self.r = np.array(tmp)
            self.r = np.reshape(self.r,(3,1))
    

    def update_matrix_callback(self, msg):
        ''' Update the feed-back and feed-forward control matrix '''
        col = len(msg.data)/msg.row
        Fa = [ msg.data[(i*col):((i+1)*col)] for i in range(msg.row) ]
        self.Fb = Fa[0:6]
        self.Ff = Fa[6:]
        rospy.set_param('control/feedback_gain', np.matrix(self.Fb).flatten())
        rospy.set_param('control/feedforward_gain', np.matrix(self.Ff).flatten())
        

    def op_mode_callback(self, msg):
        ''' Update the vehicle operation mode:
            0 = Autonomous Mode
            1 = Teleop Mode - classic guidance
            2 = Teleop Mode - inertial guidance
        '''
        if not 0 <= msg.data <= 3:
            raise ValueError(f'Operation mode {msg.data} not supported, only [1-2-3] mode. See the documentation') 
        self.op_mode = msg.data


        #def __vector2matrix(self, vect, row):
        #    ''' Convert vector to matrix ''' 
        #    col = len(vect)/row
        #    return [ vect[(i*col):((i+1)*col)] for i in range(row) ]



if __name__ == '__main__':
    vc = VehicleController()
    vc.loop() 