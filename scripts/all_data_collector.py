#! /usr/bin/env python3
# -*- coding: utf-8 -*-

#  GNU License (C) 2021  ELTOS
#  This program is developed for Unical by Unical's researchers.
#  For a copy of the complete license please contact  {franco.torchiaro}{elqemmah.ay}@dimes.unical.it.

import  rospy
import numpy as np
import math

from std_msgs.msg import Header, String, Float64
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Vector3Stamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler


import random as random
import time as time
import matplotlib.pyplot   as plt
from asv.msg import Vector, Matrix 
from geometry_msgs.msg import Quaternion,Vector3
from scipy import integrate
from scipy import io

import sys
import os
sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/')
import WGS84


class Identification_manager:

    ''' Init node '''
    def __init__(self):
        # INIT NODE
        rospy.init_node('identification_manager')

        ##################### THIS NODE EXPECTS A thrusters_input.mat FILE 

        # PREFIX
        #topic_prefix = rospy.get_param('~topic_prefix')
        topic_prefix = '/asv1'

        self.lat0 = 39.330791
        self.lon0 = 16.250741
        self.alt0 = 0.1
        # PARAMETERS
        self.freq = rospy.get_param('identification/freq', 100)
        self.time_window = rospy.get_param('identification/time_window', 15)
        self.lla_ref = (self.lat0, self.lon0, self.alt0)

        rospy.Subscriber('/asv1/fix_ublox', NavSatFix, self.gps_ublox_callback)
        rospy.Subscriber('/asv1/vehicle/state', Vector, self.gps_position_callback)
        rospy.Subscriber('/asv1/fix_velocity', Vector3Stamped, self.gps_velocity_callback)
        rospy.Subscriber('/asv1/imu', Imu, self.imu_callback)
        rospy.Subscriber('/asv1/imu_vect_gyro',Vector3,self.imu_gyro_callback)
        rospy.Subscriber('/asv1/low_level/vehicle/tau', String, self.real_thrust_callback)


        rospy.Subscriber('/asv1/controller/state', Vector, self.state_controller_callback)
        rospy.Subscriber('/asv1/vehicle/reference', Vector, self.state_g_callback)

        
        self.tau = np.array((0,0,0))
        self.tau = np.reshape(self.tau,(3,1))
        self.position_gps = np.zeros((3,1))
        self.position_gps_ublox = np.zeros((3,1))
        self.controller_state = np.zeros((3,1))
        self.admissible_reference = np.zeros((3,1))
        self.orietation_imu = np.zeros((4,1))
        self.orientation_yaw = 0.0
        self.acc_imu = np.zeros((3,1))
        self.gyro = np.zeros((3,1))
        self.normalized_speed = np.zeros((3,1))

    def state_controller_callback(self,msg):
        tmp = msg.data
        self.controller_state = np.zeros((3, 1))
        self.controller_state = np.reshape(np.array((float(tmp[0]),float(tmp[1]),float(tmp[2]))),(3,1))

    def state_g_callback(self,msg):
        """
        # Current computed admissible reference "for evaluation purposes" 
        """
        tmp = msg.data
        self.admissible_reference = np.reshape(np.array((float(tmp[0]),float(tmp[1]),float(tmp[2]))),(3,1))

    def gps_ublox_callback(self, msg):
        """
        # Current position given directly by the GPS sensor 
        """
        lla = (msg.latitude, msg.longitude, msg.altitude)
        self.lat0 = msg.latitude
        self.lon0 = msg.longitude
        self.lla_ref = (self.lat0, self.lon0, self.alt0)
        # Convert LLA to ENU
        #enu = WGS84.lla2enu(lla,self.lla_ref)
        #self.position_gps = np.array((enu[0],enu[1],enu[2]))
        self.position_gps_ublox = np.array((self.lla_ref[0],self.lla_ref[1],self.lla_ref[2]))
        self.position_gps_ublox = np.reshape(self.position_gps_ublox,(3,1))
    
    def gps_position_callback(self, msg):
        """
        # Current position determined in node gps_imu_odometry 
        """
        self.position_gps = np.array((msg.data[0],msg.data[1],self.alt0))
        self.position_gps = np.reshape(self.position_gps,(3,1))
        self.orientation_yaw = msg.data[2]
        #print(self.lla_ref[0])
        
    def imu_callback(self, msg):
        """
        # Current orientation given directly in quaternion form from Vectornav IMU s
        """
        orientationx = msg.orientation.x
        orientationy = msg.orientation.y
        orientationz = msg.orientation.z
        orientationw = msg.orientation.w

        linear_acc_x = msg.linear_acceleration.x
        linear_acc_y = msg.linear_acceleration.y
        linear_acc_z = msg.linear_acceleration.z 

        #orientation_euler = euler_from_quaternion([orientationx, orientationy, orientationz, orientationw])
        #self.orietation_imu = np.array((orientation_euler[0], orientation_euler[1], orientation_euler[2]))
        self.orietation_imu = np.array((orientationx, orientationy, orientationz, orientationw))
        self.orietation_imu = np.reshape(self.orietation_imu, (4,1))

        self.acc_imu = np.array((linear_acc_x, linear_acc_y, linear_acc_z))
        self.acc_imu = np.reshape(self.acc_imu, (3,1))

    def gps_velocity_callback(self, msg):
        """
        # Current computed form GPS velocities in ENU frame 
        """
        self.normalized_speed = np.array((float(msg.vector.x),float(msg.vector.y),float(msg.vector.z)))
        self.normalized_speed = np.reshape(self.normalized_speed,(3,1))


    def imu_gyro_callback(self, msg):
        """
        # Current gyroscope measurments given from the Vectornav IMU 
        """
        self.gyro = np.array((float(msg.x),float(msg.y),float(msg.z)))
        self.gyro = np.reshape(self.gyro,(3,1))

    def real_thrust_callback(self, msg):
        tmp = msg.data
        tmp = tmp.split(' ')
        self.tau = np.array((float(tmp[0]),float(tmp[1]),float(tmp[2])))
        self.tau = np.reshape(self.tau,(3,1))


    def collect_data(self, time_duration):

        """
        # Real data callback collector, could be optimized 
        """ 
        rate = rospy.Rate(self.freq)
        position = np.zeros((3, time_duration))
        position_ublox = np.zeros((3, time_duration))
        orientation =  np.zeros((4, time_duration))
        orientation_yaw = np.zeros((1, time_duration))
        acceleration =  np.zeros((3, time_duration))
        v_speed = np.zeros((3, time_duration))
        tau_applied = np.zeros((3, time_duration))
        gyro = np.zeros((3, time_duration))
        controller_state = np.zeros((3, time_duration)) 
        admissible_reference = np.zeros((3, time_duration)) 


        counter = 0

        while not rospy.is_shutdown() and counter < time_duration:

            #self.thruster_pub.publish(tau[:,counter].tolist()) # apply command to simulated vehicle

            #collected_speed = self.get_vehicle_speed()
            position[0, counter] = self.position_gps[0]
            position[1, counter] = self.position_gps[1]
            position[2, counter] = self.position_gps[2]

            position_ublox[0, counter] = self.position_gps_ublox[0]
            position_ublox[1, counter] = self.position_gps_ublox[1]
            position_ublox[2, counter] = self.position_gps_ublox[2]
            
            orientation[0, counter] = self.orietation_imu[0]
            orientation[1, counter] = self.orietation_imu[1]
            orientation[2, counter] = self.orietation_imu[2]
            orientation[3, counter] = self.orietation_imu[3]

            orientation_yaw[0, counter] = self.orientation_yaw

            acceleration[0, counter] = self.acc_imu[0]
            acceleration[1, counter] = self.acc_imu[1]
            acceleration[2, counter] = self.acc_imu[2]

            gyro[0, counter] = self.gyro[0]
            gyro[1, counter] = self.gyro[1]
            gyro[2, counter] = self.gyro[2]

            #rospy.loginfo(v_speed.shape)
            v_speed[0, counter] = self.normalized_speed[0]
            v_speed[1, counter] = self.normalized_speed[1]
            v_speed[2, counter] = self.normalized_speed[2]

            tau_applied[0, counter] = self.tau[0]
            tau_applied[1, counter] = self.tau[1]
            tau_applied[2, counter] = self.tau[2]

            controller_state[0, counter] = self.controller_state[0]
            controller_state[1, counter] = self.controller_state[1]
            controller_state[2, counter] = self.controller_state[2]

            admissible_reference[0, counter] = self.admissible_reference[0]
            admissible_reference[1, counter] = self.admissible_reference[1]
            admissible_reference[2, counter] = self.admissible_reference[2]

            rate.sleep()
            counter = counter + 1

        return (position, position_ublox, orientation, orientation_yaw, acceleration, gyro, v_speed, tau_applied, controller_state, admissible_reference)


    def loop(self):



        rospy.loginfo("Identification Node: Started")

        rospy.sleep(2)

        rospy.loginfo("Starting data collector")

        while not rospy.is_shutdown():

        ############ COLLECT SPEED DATA 

            time_duration = 60

            samples = int(time_duration/(0.01))

            (position, position_ublox, orientation, orientation_yaw, acceleration, gyro, v_speed, tau, controller_state, admissible_reference) = self.collect_data(samples)

            t = np.linspace(0.0, time_duration, len(position[0,:]))

            ########### SAVE DATA
            data = {"position": position, "position_gps": position_ublox, \
            "orientation": orientation, "orientation_yaw": orientation_yaw, \
            "acceleration": acceleration, "gyro": gyro, "v_speed":v_speed, \
            "tau":tau, "t":t, "controller_state":controller_state, "admissible_reference": admissible_reference}
            io.savemat("/home/udoo/ares_ros_ws/DCG5_ANTICOLLISION.mat", data)
            return 0


            


if __name__ == '__main__':
    rospy.set_param('identification/freq', 100)
    rospy.set_param('identification/time_window', 60)
    rospy.set_param('identification/torque', [0,0,0])
    id_m = Identification_manager()
    id_m.loop()
