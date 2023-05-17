#! /usr/bin/env python3
# -*- coding: utf-8 -*-
from asv.msg import Matrix, Vector
import time
import rospy 

x2 = lambda x: [i*0.4 for i in x]
''' Init node '''
# INIT NODE
rospy.init_node('gain_modifier')
feedback_gain = x2([ 90.9624424866361, 0.0, 0.0, 120.797471311055, 0.0, 0.0, \
                  0.0, 90.9220551566911, 0.0, 0.0, 120.787310840082, 0.0, \
                  0.0, 0.0, 4.21873668386763, 0.0, 0.0, 5.43106263122312])
                    #4.21873668386763           5.43106263122312

feedforward_gain = x2([ 11.9796, 0.0, 0.0,     \
                     0.0, 11.9722,   0.0,   \
                     0.0, 0.0, 0.5555])
                        # 0.5555

allocation_gain = x2([ 0.0, 0.0, 0.0, \
                        0.0, 0.0, 0.0, \
                        0.0, 0.0, 0.0, \
                        0.0, 0.0, 0.0])

print(feedback_gain)

gain_modifier_pub = rospy.Publisher('/asv1/control_matrix_update', Matrix , queue_size = 1)
allocation_modifier_pub = rospy.Publisher('/asv1/allocation_matrix_update', Matrix , queue_size = 1)
ref_pub = rospy.Publisher('/asv1/vehicle/reference', Vector, queue_size = 1)
gains_pub = rospy.Publisher('/asv1/gains/update', Vector, queue_size = 1)


gm_msg = Matrix()
gm_msg.Fb = feedback_gain
gm_msg.Fa = feedforward_gain

ref_msg = Vector()
ref_msg.data = [-6.0,12.0, 1.57]

#alloc_msg = Matrix()
#alloc_msg.Fa = allocation_matrix

gains_msg = Vector()
gains_msg.data = [0, 0, 7.2]

#gain_modifier_pub.publish(gm_msg)
ref_pub.publish(ref_msg)

first = True

while not rospy.is_shutdown():
    time.sleep(1)
    if first:
        ref_pub.publish(ref_msg)
        #gain_modifier_pub.publish(gm_msg)
        #allocation_modifier_pub(alloc_msg)
        #gains_pub.publish(gains_msg)
        first = False
