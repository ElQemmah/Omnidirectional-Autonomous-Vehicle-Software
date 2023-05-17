#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import actionlib
import matplotlib.pyplot as plt

from asv.msg import Vector
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Joy

class Plot:
    ''' Init node '''
    def __init__(self):
        # INIT NODE
        rospy.init_node('joystick_teleop')
        self.rate = rospy.Rate(rospy.get_param('sensors/state_freq', 50))
        self.op_mode = 0 
        self.command = None
        self.send_command = False
        self.saturation = 50
        self.local_info = rospy.Subscriber('/joy', Joy, self.command_callback)
        self.global_info = rospy.Publisher('/asv1/teleop/tau', Vector, queue_size=1)
        self.command_info = rospy.Publisher('/asv1/teleop/op_mode', Int32, queue_size=1)  #### SHOULD BE MOVED TO LOW LEVEL THRUSTER MANAGER
        self.change_sat = rospy.Subscriber('/asv1/changesat', Int32, self.saturation_callback)

        # 0 1 2 per op mode   0 AUTO, 1 CLASSIC, 2 INERTIAL

    def saturation_callback(self, msg):
        if not 0 <= msg.data <= 70:
            raise ValueError(f'Saturation value of {msg.data} is not supported, please choose a value lower than 70 [N]')
        self.saturation = msg.data


    
    def command_callback(self, msg):      
        tmp = msg.axes 
        tmp2 = msg.buttons
        self.send_command = False
        if(tmp2[0] == 1): # SQUARE button Corresponds to AUTO
            self.op_mode = 0
            self.send_command = True
        if(tmp2[1] == 1): # X button Corresponds to CLASSIC
            self.op_mode = 1
            self.send_command = True
        if(tmp2[2] == 1): # CIRCLE button Corresponds to INERTIAL
            self.op_mode = 2
            self.send_command = True

        self.command = [-self.saturation*round(tmp[1],2), -self.saturation*round(tmp[0],2), self.saturation*0.25*round(tmp[2],2)]

    
        
    def loop(self):
        while not rospy.is_shutdown():
            if(self.command is None):
                command = rospy.wait_for_message('/joy', Joy)
                command = command.axes
                self.command = [-self.saturation *round(command[1],2), -self.saturation *round(command[0],2), self.saturation*0.25*round(command[2],2)]
            self.global_info.publish(self.command)
            if(self.send_command==True):
                self.command_info.publish(self.op_mode)
            self.rate.sleep()
            #return 


        





if __name__ == '__main__':

    plot_pos = Plot()
    plot_pos.loop()
    


