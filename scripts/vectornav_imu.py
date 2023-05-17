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

### Library imports
import sys
import os
sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/vectlib/')
import vectornav_interface as VECTORNAV
from vectornav_packet import Vn_packet

#import WGS84

### MSG imports
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion,Vector3



class ImuVectornav:
    def __init__(self):
        ''' Init node '''
        # INIT NODE
        rospy.init_node('imu_vectornav')

        # IMU SENSOR INTERFACING
        self.vect = VECTORNAV.Vn_interface(False) # True for debug mode
        self.port = '/dev/ttyUSB2'
        self.baudrate = 115200
        self.vect.connect_to_device(self.port, self.baudrate)
        #self.vect.configure_device_output()
        """
        while True:
            v_board.read_next_packet()
            v_board.measure(1000)
        v_board.close_connection()
        """
        self.rate = rospy.Rate(rospy.get_param('sensors/state_freq', 100))
        self.debug = False

        #self.last_packet = Vn_packet()

        # PARAMETERS
        self.imu_state = Imu()
        self.imu_state.header.frame_id = 'imu_vectornav'
        self.imu_state.header.stamp = rospy.Time.now()


        self.imu_state.orientation = Quaternion()
        self.imu_state.orientation.x = 0.0
        self.imu_state.orientation.y = 0.0
        self.imu_state.orientation.z = 0.0
        self.imu_state.orientation.w = 0.0
        self.imu_state.angular_velocity = Vector3()
        self.imu_state.angular_velocity.x = 0.0
        self.imu_state.angular_velocity.y = 0.0
        self.imu_state.angular_velocity.z = 0.0
        self.imu_state.linear_acceleration = Vector3()
        self.imu_state.linear_acceleration.x = 0.0
        self.imu_state.linear_acceleration.y = 0.0
        self.imu_state.linear_acceleration.z = 0.0

        self.imu_state_gyro = Vector3()
        self.imu_state_gyro.x = 0.0
        self.imu_state_gyro.y = 0.0
        self.imu_state_gyro.z = 0.0

        # TOPICS
        topic_prefix = rospy.get_param('~topic_prefix')
        #topic_prefix = '/asv1'
        self.imu_state_pub = rospy.Publisher(f'{topic_prefix}/imu', Imu, queue_size=1)
        self.imu_state_gyro_pub = rospy.Publisher(f'{topic_prefix}/imu_vect_gyro', Vector3, queue_size=10)


    def loop(self):
        ''' Life cycle '''
        print("Starting reading process")
        while not rospy.is_shutdown():

            self.last_packet = self.vect.read_next_packet()
           
            if (self.last_packet.packetType() == '$VNYMR'):
                self.imu_state.orientation.x = self.last_packet.orientationQuaternion()[0]
                self.imu_state.orientation.y = self.last_packet.orientationQuaternion()[1]
                self.imu_state.orientation.z = self.last_packet.orientationQuaternion()[2]
                self.imu_state.orientation.w = self.last_packet.orientationQuaternion()[3]
                self.imu_state.linear_acceleration.x = self.last_packet.freeAcceleration()[0]
                self.imu_state.linear_acceleration.y = self.last_packet.freeAcceleration()[1]
                self.imu_state.linear_acceleration.z = self.last_packet.freeAcceleration()[2]
                self.imu_state_gyro.x = self.last_packet.gyroscopeData()[0]
                self.imu_state_gyro.y = self.last_packet.gyroscopeData()[1]
                self.imu_state_gyro.z = self.last_packet.gyroscopeData()[2]
                if self.debug:
                    print(self.last_packet.orientationQuaternion())
                    print(self.last_packet.freeAcceleration())
            self.imu_state.header.stamp = rospy.Time.now()
            self.imu_state_pub.publish(self.imu_state)   
            self.imu_state_gyro_pub.publish(self.imu_state_gyro)
            self.rate.sleep()

        self.vect.close_connection()


if __name__ == '__main__':
    gio = ImuVectornav()
    gio.loop()


