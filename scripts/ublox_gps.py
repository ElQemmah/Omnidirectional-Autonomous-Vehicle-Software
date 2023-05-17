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



allocation : { 'allocation_matrix' : [0.35355339, -0.35355339, -0.69444444, 0.35355339, 0.35355339, 0.69444444, -0.35355339, 0.35355339, -0.69444444, 0.35355339, -0.35355339, 0.69444444]}



import rospy

### Library imports
import sys
import os


sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/ublib/')
import ublox_interface as UBLOX
#import WGS84

### MSG imports
from sensor_msgs.msg import NavSatFix, Imu, NavSatStatus
from geometry_msgs.msg import Vector3Stamped
from asv.msg import Vector

class GpsUblox:
    def __init__(self):
        ''' Init node '''
        # INIT NODE
        rospy.init_node('gps_ublox')

        # GPS SENSOR INTERFACING
        self.ubx_i = UBLOX.Ubx_interface()
        self.ubx_i.connect('/dev/ttyACM1')

        # PARAMETERS
        self.gps_position = NavSatFix()
        self.gps_position.header.frame_id = 'gps_ublox'
        self.gps_position.header.stamp = rospy.Time.now()
        self.gps_position.status.service = 1
        self.rate = rospy.Rate(rospy.get_param('sensors/state_freq', 100))
        self.debug = True

        # Base coordinates in LLA[dd] frame
        lat0 = rospy.get_param('map/lat0', 39.30367267)
        lon0 = rospy.get_param('map/lon0', 16.25211817)
        alt0 = rospy.get_param('map/alt0', 0.1)
        #self.lla_ref = (lat0, lon0, alt0)

        """
        self.gps_position.latitude = 39.3665100
        self.gps_position.longitude = 16.2264010
        self.gps_position.altitude = 0.1
        """
        
        self.gps_speed = Vector3Stamped()

        # TOPICS
        #topic_prefix = rospy.get_param('~topic_prefix')
        #topic_prefix = '/asv1'
        self.position_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
        self.speed_pub = rospy.Publisher('fix_velocity', Vector3Stamped, queue_size=1)



    def loop(self):
        ''' Life cycle '''
        print("Starting reading process")
        while not rospy.is_shutdown():
            ll = self.ubx_i.lat_lon
            if (ll[0] != '' and ll[1] != ''):
                self.lat0 = ll[0]
                self.lon0 = ll[1]
                self.gps_position.latitude = self.lat0
                self.gps_position.longitude = self.lon0
                self.gps_position.altitude = float(self.ubx_i.alt0)
            ned_vel = [self.ubx_i.velN, self.ubx_i.velE, self.ubx_i.velD]
            if not ('' in ned_vel):
                self.gps_speed.vector.x = ned_vel[1]
                self.gps_speed.vector.y = ned_vel[0]
                self.gps_speed.vector.z = ned_vel[2]    
            if self.debug:
                print(self.gps_position.status)
                print(self.ubx_i.cog)


            self.gps_position.header.stamp = rospy.Time.now()
            self.position_pub.publish(self.gps_position)
            self.speed_pub.publish(self.gps_speed)
            self.rate.sleep()

        self.ubx_i.close()


if __name__ == '__main__':
    gio = GpsUblox()
    gio.loop()


