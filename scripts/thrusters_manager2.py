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

import rospy
import math
import numpy as np
from asv.msg import Vector, Matrix
from std_msgs.msg import String, Int32
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
import serial

PORT = "/dev/ttyACM2"
BAUDRATE = 115200

class ThrustersManager2:

    def __init__(self, port=PORT, baudrate = BAUDRATE):
        ''' Init node '''
        # INIT NODE
        rospy.init_node('thrusters_manager_node')

        # PARAMETERS
        Tc = rospy.get_param('control/Tc', 0.05)
        self.rate = rospy.Rate(1/Tc)
        self.__seq = 0
        self.Cb = 0.00016 # Rotor constant
        #self.Ti = np.zeros((self.N_thruster,1))#
        self.T_uvr = [0.0, 0.0, 0.0]
        self.T_1234 = [0.0, 0.0, 0.0, 0.0]
        self.op_mode = 0 #1 # Default: Classic Teleop Mode

        self.low_level = serial.Serial(port, baudrate)
        
        self.th = None


        self.lc = 2 # smoothness width used in compensation 

        # Deadzone left and right value for first thruster
        self.zmdT1 = 2.2 
        self.zmeT1 = -2.7 
        # Deadzone left and right value for second thruster
        self.zmdT2 = 2.1 
        self.zmeT2 = -2.6 
        # Deadzone left and right value for third thruster
        self.zmdT3 = 2.1 
        self.zmeT3 = -2.4 
        # Deadzone left and right value for fourth thruster
        self.zmdT4 = 2.4 
        self.zmeT4 = -2.4 
        # Right and left slopes of the output (x-axis u, y-axis u_deadzone)
        self.md = 1
        self.me = 1 

        if(self.th is None):
            rospy.sleep(1.5)
            msg = rospy.wait_for_message('vehicle/state', Vector)
            tmp = msg.data
            x =  np.reshape(np.array((tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5])),(6,1))
            x = np.reshape(x, (6,1))
            self.th = x[2]

        self.R = lambda th: [[ np.cos(th), np.sin(th), 0],\
                             [-np.sin(th), np.cos(th), 0],\
                             [ 0,          0,          1]]
        

        a = rospy.get_param('physics/thrusters/alpha', math.pi/4)
        r = rospy.get_param('physics/thrusters/l_center', 0.18)
        sigma = rospy.get_param('physics/thrusters/allocation_matrix', self.__build_allocation_matrix(a,r))
        self.sigma_pinv = (-1)*np.linalg.pinv(sigma)
        
        # TOPICS
        topic_prefix = rospy.get_param('~topic_prefix')
        rospy.Subscriber('teleop/tau', Vector, self.teleop_thrust_callback)
        rospy.Subscriber('vehicle/tau', Vector, self.vehicle_thrust_callback)
        rospy.Subscriber('teleop/op_mode', Int32, self.op_mode_callback) # modify message type in OpMode enum
        rospy.Subscriber('vehicle/state', Vector, self.state_callback)
        rospy.Subscriber('allocation_matrix_update', Matrix, self.update_matrix_callback)
        rospy.Subscriber('low_level/vehicle/tau', String, self.real_thrust_callback)

        self.LL_tau_pub = rospy.Publisher('low_level/vehicle/tau', String, queue_size=30)

        # Topic used to get applied thrust for identification 
        #self.identification_pub = rospy.Publisher(f'{topic_prefix}/input_identification',Vector,queue_size=10)
        #self.thruster_pub = [ rospy.Publisher(f'{topic_prefix}/thrusters/{i+1}/input', FloatStamped , queue_size=1) for i in range(self.N_thruster) ]


    def loop(self):
        ''' Life cycle
        Update command to thruster each update_freq[Hz]. For publish only when the msg come
        move the build phase of Ti_msg (now in loop method) in the 2 callback.
        '''
        while not rospy.is_shutdown():
            msg_str = ''
            msg_str_pub = ''
            #self.compute_deadzonecompensation() 
            for value in self.T_1234:
                msg_str += f'{round(value,2)};'
                msg_str_pub += f'{round(value,2)}'
                msg_str_pub += ' '
            #msg_str = "2.25;2.25;-2.25;-2.25;" DEAD BAND
            #print(msg_str)
            nbyte = self.low_level.write(msg_str.encode())
            self.LL_tau_pub.publish(msg_str_pub)
            #rospy.loginfo(nbyte)
            self.rate.sleep()


    def real_thrust_callback(self, msg):
        tmp = msg.data
        tmp = tmp.split(' ')
        tau = [float(tmp[0]),float(tmp[1]),float(tmp[2]),float(tmp[3])]
        self.T_1234[0] = tau[0]
        self.T_1234[1] = tau[1]
        self.T_1234[2] = tau[2]
        self.T_1234[3] = tau[3]


    def vehicle_thrust_callback(self, msg):
        ''' Received command from control module '''
        #msg.data: { T_xyz }
        
        #print('vehicle/thrust callback')
        T_1234 = [0.0, 0.0, 0.0, 0.0]

        if self.op_mode > 0: # Teleop Mode {1,2}
            T_1234[0] = 0.0 
            T_1234[1] = 0.0 
            T_1234[2] = 0.0 
            T_1234[3] = 0.0 
            #print(self.op_mode)
            #return # Skip vehicle command
        else: # Autonomous Mode {T_xyz}
            """
            tmp = list(msg.data)
            
            
            tmp[0] = -msg.data[1] 
            tmp[1] = msg.data[0]
            tmp[2] = msg.data[2]

            print(tmp)
            """
            self.T_uvr = np.dot(self.R(self.th), msg.data)
            
            self.T_uvr[0] =  self.T_uvr[0]
            self.T_uvr[1] =  self.T_uvr[1]
            self.T_uvr[2] =  self.T_uvr[2]
            #print(self.T_uvr)
            T_1234 = np.dot(self.sigma_pinv, self.T_uvr)
            #print(T_1234)
        #self.identification_pub.publish(msg.data)
        
        #self.T_uvr = np.dot(self.R(self.th), msg.data)
        #T_1234 = np.dot(self.sigma_pinv, self.T_uvr)

        self.T_1234[0] = float(T_1234[0])
        self.T_1234[1] = float(T_1234[1])
        self.T_1234[2] = float(T_1234[2])
        self.T_1234[3] = float(T_1234[3])


    def teleop_thrust_callback(self, msg):
        ''' Received command from teleoperation module
        msg.data: { T_uvr ; T_xyz }
        '''
        if self.op_mode == 0: # Autonomous Mode {0}
            #self.T_uvr = msg.data
            return # Skip radio command
        if self.op_mode == 1: # Classic Teleop Mode {T_uvr}
            self.T_uvr = msg.data
        elif self.op_mode == 2: # Inertial Teleop Mode {T_xyz}
        #    self.T_uvr = msg.data#
            #self.T_uvr = msg.data
            self.T_uvr = np.dot(self.R(self.th), msg.data)
            #self.identification_pub.publish(T_uvr.tolist())
        #self.Ti = np.dot(self.sigma_pinv, T_uvr)
        T_1234 = np.dot(self.sigma_pinv, self.T_uvr)
        self.T_1234[0] = float(T_1234[0])
        self.T_1234[1] = float(T_1234[1])
        self.T_1234[2] = float(T_1234[2])
        self.T_1234[3] = float(T_1234[3])



    def op_mode_callback(self, msg):
        ''' Update the vehicle operation mode:
        0 = Autonomous Mode
        1 = Teleop Mode - classic guidance
        2 = Teleop Mode - inertial guidance
        '''
        if not 0 <= msg.data <= 3:
            raise ValueError(f'Operation mode {msg.data} not supported, only [1-2-3] mode. See the documentation')
        self.op_mode = msg.data


    def __build_allocation_matrix(self, a, r):
        ''' Build the allocation matrix and his pseudo-inverse form '''
        s = math.sin(a)
        c = math.cos(a)
        return [[ c,  c, -c, -c],\
                 [-s,  s,  s, -s],\
                 [-r,  r, -r,  r]]

    def state_callback(self, msg):
        ''' Update the heading angle of the vehicle '''
        self.th = msg.data[2] 


    def update_matrix_callback(self, msg):
        ''' Update the allocation matrix and his pseudo-inverse form '''
        col = len(msg.data)/msg.row
        sigma = [ msg.data[(i*col):((i+1)*col)] for i in range(msg.row) ]
        rospy.set_param('physics/thrusters/allocation_matrix', sigma)
        self.sigma_pinv = np.linalg.pinv(sigma)

    def compute_deadzonecompensation(self):

        if(self.T_1234[0]>=self.lc):
            self.T_1234[0] = (self.T_1234[0]/self.md) + self.zmdT1

        elif(self.T_1234[0]<=self.lc and self.T_1234[0]>=0):
            self.T_1234[0] = ((self.zmdT1 + (self.lc/self.md))/self.lc)*self.T_1234[0]

        elif(self.T_1234[0]<=-self.lc):
            self.T_1234[0] = (self.T_1234[0]/self.me) + self.zmdT1

        elif(self.T_1234[0]>=-self.lc and self.T_1234[0]<=0):
            self.T_1234[0] = ((self.zmeT1 + (self.lc/self.me))/self.lc)*self.T_1234[0]


        if(self.T_1234[1]>=self.lc):
            self.T_1234[1] = (self.T_1234[1]/self.md) + self.zmdT2

        elif(self.T_1234[1]<=self.lc and self.T_1234[1]>=0):
            self.T_1234[1] = ((self.zmdT2 + (self.lc/self.md))/self.lc)*self.T_1234[1]

        elif(self.T_1234[1]<=-self.lc):
            self.T_1234[1] = (self.T_1234[1]/self.me) + self.zmdT2

        elif(self.T_1234[1]>=-self.lc and self.T_1234[1]<=0):
            self.T_1234[1] = ((self.zmeT2 + (self.lc/self.me))/self.lc)*self.T_1234[1]


        if(self.T_1234[2]>=self.lc):
            self.T_1234[2] = (self.T_1234[2]/self.md) + self.zmdT3

        elif(self.T_1234[2]<=self.lc and self.T_1234[2]>=0):
            self.T_1234[2] = ((self.zmdT3 + (self.lc/self.md))/self.lc)*self.T_1234[2]

        elif(self.T_1234[2]<=-self.lc):
            self.T_1234[2] = (self.T_1234[2]/self.me) + self.zmdT3

        elif(self.T_1234[2]>=-self.lc and self.T_1234[2]<=0):
            self.T_1234[2] = ((self.zmeT3 + (self.lc/self.me))/self.lc)*self.T_1234[2]
    

        if(self.T_1234[3]>=self.lc):
            self.T_1234[3] = (self.T_1234[3]/self.md) + self.zmdT4

        elif(self.T_1234[3]<=self.lc and self.T_1234[3]>=0):
            self.T_1234[3] = ((self.zmdT4 + (self.lc/self.md))/self.lc)*self.T_1234[3]

        elif(self.T_1234[3]<=-self.lc):
            self.T_1234[3] = (self.T_1234[3]/self.me) + self.zmdT4

        elif(self.T_1234[3]>=-self.lc and self.T_1234[3]<=0):
            self.T_1234[3] = ((self.zmeT4 + (self.lc/self.me))/self.lc)*self.T_1234[3]    


    def update_matrix_callback(self, msg):
        ''' Update the thrusters allocation matrix '''
        self.Sigma = msg.Fa
        self.Sigma = np.array(self.Sigma).reshape(4,3)



if __name__ == '__main__':
    tm = ThrustersManager2()
    tm.loop()
