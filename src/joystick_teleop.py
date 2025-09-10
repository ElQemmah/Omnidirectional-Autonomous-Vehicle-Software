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
import matplotlib.pyplot as plt

from asv.msg import Vector
from sensor_msgs.msg import Joy

class Plot:
    ''' Init node '''
    def __init__(self):
        # INIT NODE
        rospy.init_node('joystick_teleop')
        self.rate = rospy.Rate(rospy.get_param('sensors/state_freq', 50))
        self.command = None
        self.local_info = rospy.Subscriber('/joy', Joy, self.command_callback)
        self.global_info = rospy.Publisher('/asv3/vehicle/tau', Vector, queue_size=1)


    
    def command_callback(self, msg):      
        tmp = msg.axes 
        #self.position = np.array([tmp[0],tmp[1]])
        self.command = [-50*tmp[1], -50*tmp[0], 0*tmp[2]]
        #self.position = np.reshape(self.position, (2,1))
    
        
    def loop(self):
        while not rospy.is_shutdown():
            if(self.command is None):
                command = rospy.wait_for_message('/joy', Joy)
                command = command.axes
                self.command = [-50*command[1], -50*command[0], 0*command[2]]
            self.global_info.publish(self.command)
            self.rate.sleep()
            #return 


        





if __name__ == '__main__':

    plot_pos = Plot()
    plot_pos.loop()
    


