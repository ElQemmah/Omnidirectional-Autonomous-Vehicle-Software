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
import time
import numpy as np 

class Vn_packet:

    def __init__(self, msg):
        msg = msg.decode("ascii")
        fields = msg[0:-3].split(',')

        # " Yaw [0]   Pitch[1]    Roll[2]
        self.orientation = self.acc = [float(fields[1]), float(fields[2]), \
                    float(fields[3])]

        #self.magnetometer = self.acc = [float(fields[4]), float(fields[5]), \
        #            float(fields[6])]
        
        self.acc = [float(fields[7]), float(fields[8]), \
                    float(fields[9])]

        self.type_packet = fields[0]

        self.gyros = self.acc = [float(fields[10]), float(fields[11]), \
                   float(fields[12])]
        

        self.time = round(time.time() * 1000)
        
        pass

    def containsOrientation(self):
        return True

    def packetType(self):
        return self.type_packet

    def orientationQuaternion(self):
        """
            Convert an Euler angle to a quaternion.
            
            Input
                :param roll: The roll (rotation around x-axis) angle in radians.
                :param pitch: The pitch (rotation around y-axis) angle in radians.
                :param yaw: The yaw (rotation around z-axis) angle in radians.
            
            Output
                :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        roll = self.orientation[2]*0
        pitch = self.orientation[1]*0
        yaw =  (self.orientation[0]*np.pi)/180
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
        return [float(qx), float(qy), float(qz), float(qw)]

    def orientationEuler(self):
        return self.orientation

    def gyroscopeData(self):
        return self.gyros

    def containsLatitudeLongitude(self):
        return False

    def latitudeLongitude(self):
        pass

    def containsAltitude(self):
        return False

    def altitude(self):
        pass

    def containsVelocity(self):
        return False

    def containsFreeAcceleration(self):
        return True

    def containsSampleTimeFine(self):
        return False

    def sampleTimeFine(self):
        return self.time

    def freeAcceleration(self):
        return self.acc
