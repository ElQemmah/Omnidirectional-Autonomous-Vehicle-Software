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
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from asv.msg import Vector
from std_msgs.msg import Header, Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Vector3Stamped
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState


import sys
import os
sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/')
import WGS84


class GpsImuOdometry:

    def __init__(self, sim=False):
        ''' Init node '''
        # INIT NODE
        rospy.init_node('gps_imu_odometry_copy')

        # PARAMETERS
        self.odom = Odometry()
        self.rate = rospy.Rate(rospy.get_param('sensors/state_freq', 100))
        self.__seq = 0
        self.debug = False
        self.theta = 0
        self.data = {'x':[], 'y':[], 'z':[], 'th':[], 'vx':[], 'vy':[], 'vth':[]}
        self.sim = sim
        #if self.sim:
            # Use Gazebo
        #    self.imu_frame_type = rospy.get_param('~imu_frame_type', 'NWU')
        #else:
            # Use Real Imu
        self.imu_frame_type = rospy.get_param('~imu_frame_type', 'NED')

        # Base coordinates in LLA[dd] frame
        lat0 = rospy.get_param('map/lat0', 39.3665100) #39.330791) #)#39.30367267)
        lon0 = rospy.get_param('map/lon0', 16.2264010) #16.250741) #)#16.25211817)
        alt0 = rospy.get_param('map/alt0', 265)#0.1)
        self.lla_ref = (lat0, lon0, alt0)

        # TOPICS
        self.topic_prefix = rospy.get_param('~topic_prefix')
        self.state_pub = rospy.Publisher('vehicle/state', Vector, queue_size=1)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber('fix', NavSatFix, self.gps_position_callback)
        rospy.Subscriber('fix_velocity', Vector3Stamped, self.gps_velocity_callback)
        rospy.Subscriber('imu', Imu, self.imu_callback)

        ####### VISUALIZATION PURPOSOSES
        self.state_msg = ModelState()
        self.state_msg.model_name = 'asv1'
        self.state_msg.pose.position.x = 0.0
        self.state_msg.pose.position.y = 0.0
        self.state_msg.pose.position.z = 1.2
        self.state_msg.pose.orientation.x = 0.0
        self.state_msg.pose.orientation.y = 0.0
        self.state_msg.pose.orientation.z = 0.0
        self.state_msg.pose.orientation.w = 1.0
        self.state_msg.twist.linear.x = 0.0
        self.state_msg.twist.linear.y = 0.0
        self.state_msg.twist.linear.z = 0.0


    def gps_position_callback(self, msg):
        ''' Convert and store the received GPS position '''
        # Current LLA position
        lla = (msg.latitude, msg.longitude, msg.altitude)
        # Convert LLA to ENU
        enu = WGS84.lla2enu(lla,self.lla_ref)

        # Override odometry position
        self.odom.pose.pose.position.x = enu[0]
        self.odom.pose.pose.position.y = enu[1]
        self.odom.pose.pose.position.z = 1.2 #enu[2]

        self.state_msg.pose.position.x = self.odom.pose.pose.position.x 
        self.state_msg.pose.position.y = self.odom.pose.pose.position.y
        self.state_msg.pose.position.z = self.odom.pose.pose.position.z

        # Log data
        if self.debug:
            ecef = WGS84.lla2ecef(lla)
            ecef_ref = WGS84.lla2ecef(self.lla_ref)
            enu = WGS84.ecef2enu(ecef, self.lla_ref)

            fn = open('log.txt','a')
            fn.write('LLA: '+str(lla[0])+', '+str(lla[1])+', '+str(lla[2])+'\n')
            fn.write('ECEF: '+str(ecef[0])+', '+str(ecef[1])+', '+str(ecef[2])+'\n')
            fn.write('ECEF REF: '+str(ecef_ref[0])+', '+str(ecef_ref[1])+', '+str(ecef_ref[2])+'\n')
            fn.write('ENU: '+str(enu[0])+', '+str(enu[1])+', '+str(enu[2])+'\n')
            fn.write('-----------------------------\n')
            fn.close()

            fn = open('data.txt','a')
            fn.write(str(lla[0])+', '+str(lla[1])+', '+str(lla[2])+', '+str(enu[0])+', '+str(enu[1])+', '+str(enu[2])+'\n')
            fn.close()


    def gps_velocity_callback(self, msg): 
        ''' Convert and store the received GPS velocity  '''
        # Received GPS velocities are given in NWU frame
        self.odom.twist.twist.linear.x =  msg.vector.x #-msg.vector.y    #+
        self.odom.twist.twist.linear.y = msg.vector.y # msg.vector.x    #-
        self.odom.twist.twist.linear.z =  msg.vector.z  
        self.state_msg.twist.linear.x = self.odom.twist.twist.linear.x
        self.state_msg.twist.linear.y = self.odom.twist.twist.linear.y
        self.state_msg.twist.linear.z = self.odom.twist.twist.linear.z


    def imu_callback(self, msg):
        ''' Convert and store the received IMU position and velocity '''
        self.odom.pose.pose.orientation.x = msg.orientation.x
        self.odom.pose.pose.orientation.y = msg.orientation.y
        self.odom.pose.pose.orientation.z = msg.orientation.z
        self.odom.pose.pose.orientation.w = msg.orientation.w

        self.state_msg.pose.orientation.x = self.odom.pose.pose.orientation.x
        self.state_msg.pose.orientation.y = self.odom.pose.pose.orientation.y 
        self.state_msg.pose.orientation.z = self.odom.pose.pose.orientation.z
        self.state_msg.pose.orientation.w = self.odom.pose.pose.orientation.w

        self.odom.twist.twist.angular.x = msg.angular_velocity.x
        self.odom.twist.twist.angular.y = msg.angular_velocity.y
        self.odom.twist.twist.angular.z = msg.angular_velocity.z

        self.state_msg.twist.angular.x =  self.odom.twist.twist.angular.x
        self.state_msg.twist.angular.y =  self.odom.twist.twist.angular.y
        self.state_msg.twist.angular.z =  self.odom.twist.twist.angular.z


    def loop(self):
        ''' Life cycle '''
        while not rospy.is_shutdown():
        #for i in range(200):
            #fn = open('data.txt','a')
            current_time  = rospy.Time.now()
            self.odom.header.stamp = current_time
            self.odom.header.frame_id = 'map'
            self.odom.child_frame_id = f'{self.topic_prefix}_tf/odom'

            self.odom_pub.publish(self.odom)
            self.odom_broadcaster.sendTransform((self.odom.pose.pose.position.x,
                                            self.odom.pose.pose.position.y,
                                            self.odom.pose.pose.position.z),
                                           (self.odom.pose.pose.orientation.x,
                                            self.odom.pose.pose.orientation.y,
                                            self.odom.pose.pose.orientation.z,
                                            self.odom.pose.pose.orientation.w),
                                            current_time, self.odom.child_frame_id, self.odom.header.frame_id)
            xyz = [self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,self.odom.pose.pose.position.z]
            rpy = euler_from_quaternion([self.odom.pose.pose.orientation.x,self.odom.pose.pose.orientation.y,self.odom.pose.pose.orientation.z,self.odom.pose.pose.orientation.w])

            #print(rpy[2])
            """
            if(self.imu_frame_type == 'NWU'):
                _theta = rpy[2] + (np.pi/2) # Transform yaw from NWU (Gazebo plugin) in ENU
            elif(self.imu_frame_type == 'NED'):
                _theta = rpy[2] - np.pi# Transform yaw from NED (VectorNav imu) in ENU
            elif(self.imu_frame_type == 'ENU'):
                pass
            """
            #_theta = rpy[2] - 5.0
            _theta = rpy[2] - 1.65

            #print(_theta)

            if(_theta < 0):
                _theta = _theta + 2*np.pi
            self.theta = _theta if self.sim else (2*np.pi - _theta)

            #self.theta = (_theta*180)/(np.pi)
            
            # Publish vehicle state
            state = [xyz[0], xyz[1], self.theta, self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y, self.odom.twist.twist.angular.z]
            #state = [0*xyz[0], 0*xyz[1], self.theta, 0*self.odom.twist.twist.linear.x, 0*self.odom.twist.twist.linear.y, self.odom.twist.twist.angular.z]
            self.state_pub.publish(state)
            # Log data
            if self.debug:
                (self.data['x']).append(state[0])
                (self.data['y']).append(state[1])
                (self.data['z']).append(xyz[2])
                (self.data['th']).append(state[2])
                (self.data['vx']).append(state[3])
                (self.data['vy']).append(state[4])
                (self.data['vth']).append(state[5])
                fn = open('state.txt','a')
                fn.write(str(state)+'\n')
                fn.close()
            #fn.write(str(self.odom.pose.pose.position.x)[1:-1]+', '+str(self.odom.pose.pose.position.y)[1:-1]+', '+str(self.odom.pose.pose.position.z)[1:-1]+'\n')
            """
            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state(self.state_msg)
            except rospy.ServiceException:
                print("Service call failed")
            """
            self.rate.sleep()
            #fn.close()
        #savemat('state.mat', self.data)

    def compute_normalization_angle(self, theta_old, theta, jump_value):


        cont_angle = theta_old//(2*np.pi)

        theta_old2 = cont_angle*(2*np.pi) + theta

        if abs(theta_old- theta_old2)>jump_value and (theta_old- theta_old2)>0: 
            cont_angle += 1
        elif abs(theta_old- theta_old2)>jump_value and (theta_old- theta_old2)<0:
            cont_angle -= 1

        angle = cont_angle*(2*np.pi) + theta
        
        return angle

    def __build_header(self):
        ''' Build a std_msgs/Header.msg object '''
        header = Header()
        header.seq = self.__seq
        header.stamp = rospy.Time.now()
        header.frame_id = ''
        self.__seq += 1
        return header



if __name__ == '__main__':
    gio = GpsImuOdometry()
    gio.loop()
